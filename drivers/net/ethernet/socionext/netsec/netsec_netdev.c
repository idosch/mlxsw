/**
 * drivers/net/ethernet/socionext/netsec/netsec_netdev.c
 *
 *  Copyright (C) 2013-2014 Fujitsu Semiconductor Limited.
 *  Copyright (C) 2014-2017 Linaro Ltd. All rights reserved.
 *     Andy Green <andy.green@linaro.org>
 *     Jassi Brar <jaswinder.singh@linaro.org>
 *     Ard Biesheuvel <ard.biesheuvel@linaro.org>
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 */

#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <net/tcp.h>
#include <net/ip6_checksum.h>
#include <linux/pm_runtime.h>

#include "netsec.h"

#define WAIT_FW_RDY_TIMEOUT 50

static const u32 desc_ring_irq_status_reg_addr[] = {
	NETSEC_REG_NRM_TX_STATUS,
	NETSEC_REG_NRM_RX_STATUS,
};

static const u32 desc_ads[] = {
	NETSEC_REG_NRM_TX_CONFIG,
	NETSEC_REG_NRM_RX_CONFIG,
};

static const u32 netsec_desc_start_reg_addr_up[] = {
	NETSEC_REG_NRM_TX_DESC_START_UP,
	NETSEC_REG_NRM_RX_DESC_START_UP,
};

static const u32 netsec_desc_start_reg_addr_lw[] = {
	NETSEC_REG_NRM_TX_DESC_START_LW,
	NETSEC_REG_NRM_RX_DESC_START_LW,
};

static u32 netsec_calc_pkt_ctrl_reg_param(const struct netsec_pkt_ctrlaram
					*pkt_ctrlaram_p)
{
	u32 param = NETSEC_PKT_CTRL_REG_MODE_NRM;

	if (pkt_ctrlaram_p->log_chksum_er_flag)
		param |= NETSEC_PKT_CTRL_REG_LOG_CHKSUM_ER;

	if (pkt_ctrlaram_p->log_hd_imcomplete_flag)
		param |= NETSEC_PKT_CTRL_REG_LOG_HD_INCOMPLETE;

	if (pkt_ctrlaram_p->log_hd_er_flag)
		param |= NETSEC_PKT_CTRL_REG_LOG_HD_ER;

	return param;
}

static int netsec_netdev_load_ucode_region(struct netsec_priv *priv, u32 reg,
					   u32 addr_h, u32 addr_l, u32 size)
{
	u64 base = (u64)addr_h << 32 | addr_l;
	__le32 *ucode;
	u32 i;

	ucode = memremap(base, size * sizeof(u32), MEMREMAP_WT);
	if (!ucode)
		return -ENOMEM;

	for (i = 0; i < size; i++)
		netsec_writel(priv, reg, le32_to_cpu(ucode[i]));

	memunmap(ucode);
	return 0;
}

static int netsec_netdev_load_microcode(struct netsec_priv *priv)
{
	int err;

	err = netsec_netdev_load_ucode_region(
		priv, NETSEC_REG_DMAC_HM_CMD_BUF,
		le32_to_cpup(priv->eeprom_base + NETSEC_EEPROM_HM_ME_ADDRESS_H),
		le32_to_cpup(priv->eeprom_base + NETSEC_EEPROM_HM_ME_ADDRESS_L),
		le32_to_cpup(priv->eeprom_base + NETSEC_EEPROM_HM_ME_SIZE));
	if (err)
		return err;

	err = netsec_netdev_load_ucode_region(
		priv, NETSEC_REG_DMAC_MH_CMD_BUF,
		le32_to_cpup(priv->eeprom_base + NETSEC_EEPROM_MH_ME_ADDRESS_H),
		le32_to_cpup(priv->eeprom_base + NETSEC_EEPROM_MH_ME_ADDRESS_L),
		le32_to_cpup(priv->eeprom_base + NETSEC_EEPROM_MH_ME_SIZE));
	if (err)
		return err;

	err = netsec_netdev_load_ucode_region(
		priv, NETSEC_REG_PKT_CMD_BUF,
		0,
		le32_to_cpup(priv->eeprom_base + NETSEC_EEPROM_PKT_ME_ADDRESS),
		le32_to_cpup(priv->eeprom_base + NETSEC_EEPROM_PKT_ME_SIZE));
	if (err)
		return err;

	return 0;
}

static int netsec_init_hardware(struct netsec_priv *priv)
{
	u32 value;
	int err;

	/* set desc_start addr */
	netsec_writel(priv, netsec_desc_start_reg_addr_up[NETSEC_RING_RX],
		      upper_32_bits(priv->desc_ring[NETSEC_RING_RX].desc_phys));
	netsec_writel(priv, netsec_desc_start_reg_addr_lw[NETSEC_RING_RX],
		      lower_32_bits(priv->desc_ring[NETSEC_RING_RX].desc_phys));

	netsec_writel(priv, netsec_desc_start_reg_addr_up[NETSEC_RING_TX],
		      upper_32_bits(priv->desc_ring[NETSEC_RING_TX].desc_phys));
	netsec_writel(priv, netsec_desc_start_reg_addr_lw[NETSEC_RING_TX],
		      lower_32_bits(priv->desc_ring[NETSEC_RING_TX].desc_phys));

	/* set normal tx desc ring config */
	netsec_writel(priv, desc_ads[NETSEC_RING_TX],
		      1 << NETSEC_REG_DESC_ENDIAN);
	netsec_writel(priv, desc_ads[NETSEC_RING_RX],
		      1 << NETSEC_REG_DESC_ENDIAN);

	err = netsec_netdev_load_microcode(priv);
	if (err) {
		netif_err(priv, probe, priv->ndev,
			  "%s: failed to load microcode (%d)\n", __func__, err);
		return err;
	}

	/* start DMA engines */
	netsec_writel(priv, NETSEC_REG_DMA_TMR_CTRL, priv->freq / 1000000 - 1);
	netsec_writel(priv, NETSEC_REG_ADDR_DIS_CORE, 0);

	usleep_range(1000, 2000);

	if (!(netsec_readl(priv, NETSEC_REG_TOP_STATUS) &
	      NETSEC_TOP_IRQ_REG_CODE_LOAD_END)) {
		netif_err(priv, drv, priv->ndev, "microengine start failed\n");
		return -ENXIO;
	}
	netsec_writel(priv, NETSEC_REG_TOP_STATUS,
		      NETSEC_TOP_IRQ_REG_CODE_LOAD_END);

	value = netsec_calc_pkt_ctrl_reg_param(&priv->param.pkt_ctrlaram);

	if (priv->param.use_jumbo_pkt_flag)
		value |= NETSEC_PKT_CTRL_REG_EN_JUMBO;

	/* change to normal mode */
	netsec_writel(priv, NETSEC_REG_DMA_MH_CTRL, MH_CTRL__MODE_TRANS);
	netsec_writel(priv, NETSEC_REG_PKT_CTRL, value);

	while ((netsec_readl(priv, NETSEC_REG_MODE_TRANS_COMP_STATUS) &
		NETSEC_MODE_TRANS_COMP_IRQ_T2N) == 0)
		cpu_relax();

	return 0;
}

static void netsec_ring_irq_clr(struct netsec_priv *priv,
				unsigned int id, u32 value)
{
	netsec_writel(priv, desc_ring_irq_status_reg_addr[id],
		      value & (NETSEC_IRQ_EMPTY | NETSEC_IRQ_ERR));
}

static void netsec_napi_tx_processing(struct netsec_priv *priv)
{
	netsec_ring_irq_clr(priv, NETSEC_RING_TX, NETSEC_IRQ_EMPTY);
	netsec_clean_tx_desc_ring(priv);

	if (netif_queue_stopped(priv->ndev) &&
	    netsec_get_tx_avail_num(priv) >= NETSEC_NETDEV_TX_PKT_SCAT_NUM_MAX)
		netif_wake_queue(priv->ndev);
}

int netsec_netdev_napi_poll(struct napi_struct *napi_p, int budget)
{
	struct netsec_priv *priv = container_of(napi_p, struct netsec_priv,
						napi);
	struct net_device *ndev = priv->ndev;
	struct netsec_rx_pkt_info rx_info;
	int ret, done = 0, rx_num = 0;
	struct netsec_frag_info frag;
	struct sk_buff *skb;
	u16 len;

	netsec_napi_tx_processing(priv);

	while (done < budget) {
		if (!rx_num) {
			rx_num = netsec_get_rx_num(priv);
			if (!rx_num)
				break;
		}
		done++;
		rx_num--;
		ret = netsec_get_rx_pkt_data(priv, &rx_info, &frag, &len, &skb);
		if (unlikely(ret == -ENOMEM)) {
			netif_err(priv, drv, priv->ndev,
				  "%s: rx fail %d\n", __func__, ret);
			ndev->stats.rx_dropped++;
			continue;
		}
		dma_unmap_single(priv->dev, frag.dma_addr, frag.len,
				 DMA_FROM_DEVICE);
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, priv->ndev);

		if (priv->rx_cksum_offload_flag &&
		    rx_info.rx_cksum_result == NETSEC_RX_CKSUM_OK)
			skb->ip_summed = CHECKSUM_UNNECESSARY;

		if (napi_gro_receive(napi_p, skb) != GRO_DROP) {
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += len;
		}
	}

	if (done < budget && napi_complete_done(napi_p, done))
		netsec_writel(priv, NETSEC_REG_INTEN_SET,
			      NETSEC_IRQ_TX | NETSEC_IRQ_RX);
	return done;
}

static netdev_tx_t netsec_netdev_start_xmit(struct sk_buff *skb,
					    struct net_device *ndev)
{
	struct netsec_priv *priv = netdev_priv(ndev);
	struct netsec_tx_pkt_ctrl tx_ctrl = {};
	u16 pend_tx, tso_seg_len = 0;
	skb_frag_t *frag;
	int count_frags;
	int ret, i;

	netsec_ring_irq_clr(priv, NETSEC_RING_TX, NETSEC_IRQ_EMPTY);

	count_frags = skb_shinfo(skb)->nr_frags + 1;

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		if ((skb->protocol == htons(ETH_P_IP) &&
		     ip_hdr(skb)->protocol == IPPROTO_TCP) ||
		    (skb->protocol == htons(ETH_P_IPV6) &&
		     ipv6_hdr(skb)->nexthdr == IPPROTO_TCP))
			tx_ctrl.cksum_offload_flag = true;
		else
			skb_checksum_help(skb);
	}

	if (skb_is_gso(skb))
		tso_seg_len = skb_shinfo(skb)->gso_size;

	if (tso_seg_len > 0) {
		if (skb->protocol == htons(ETH_P_IP)) {
			ip_hdr(skb)->tot_len = 0;
			tcp_hdr(skb)->check =
				~tcp_v4_check(0, ip_hdr(skb)->saddr,
					      ip_hdr(skb)->daddr, 0);
		} else {
			ipv6_hdr(skb)->payload_len = 0;
			tcp_hdr(skb)->check =
				~csum_ipv6_magic(&ipv6_hdr(skb)->saddr,
						 &ipv6_hdr(skb)->daddr,
						 0, IPPROTO_TCP, 0);
		}

		tx_ctrl.tcp_seg_offload_flag = true;
		tx_ctrl.tcp_seg_len = tso_seg_len;
	}

	priv->tx_info[0].dma_addr = dma_map_single(priv->dev, skb->data,
						   skb_headlen(skb),
						   DMA_TO_DEVICE);
	if (dma_mapping_error(priv->dev, priv->tx_info[0].dma_addr)) {
		netif_err(priv, drv, priv->ndev,
			  "%s: DMA mapping failed\n", __func__);
		return NETDEV_TX_OK;
	}
	priv->tx_info[0].addr = skb->data;
	priv->tx_info[0].len = skb_headlen(skb);

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		frag = &skb_shinfo(skb)->frags[i];
		priv->tx_info[i + 1].dma_addr =
			skb_frag_dma_map(priv->dev, frag, 0,
					 skb_frag_size(frag), DMA_TO_DEVICE);
		priv->tx_info[i + 1].addr = skb_frag_address(frag);
		priv->tx_info[i + 1].len = frag->size;
	}

	netsec_mark_skb_type(skb, NETSEC_RING_TX);

	ret = netsec_set_tx_pkt_data(priv, &tx_ctrl, count_frags,
				     priv->tx_info, skb);
	if (ret) {
		netif_info(priv, drv, priv->ndev,
			   "set tx pkt failed %d\n", ret);
		for (i = 0; i < count_frags; i++)
			dma_unmap_single(priv->dev, priv->tx_info[i].dma_addr,
					 priv->tx_info[i].len, DMA_TO_DEVICE);
		ndev->stats.tx_dropped++;

		return NETDEV_TX_OK;
	}

	netdev_sent_queue(priv->ndev, skb->len);

	spin_lock(&priv->tx_queue_lock);
	pend_tx = netsec_get_tx_avail_num(priv);

	if (pend_tx < NETSEC_NETDEV_TX_PKT_SCAT_NUM_MAX) {
		netsec_ring_irq_enable(priv, NETSEC_RING_TX, NETSEC_IRQ_EMPTY);
		netif_stop_queue(ndev);
		goto err;
	}
	if (pend_tx <= DESC_NUM - 2) {
		netsec_ring_irq_enable(priv, NETSEC_RING_TX, NETSEC_IRQ_EMPTY);
		goto err;
	}
	netsec_ring_irq_disable(priv, NETSEC_RING_TX, NETSEC_IRQ_EMPTY);

err:
	spin_unlock(&priv->tx_queue_lock);

	return NETDEV_TX_OK;
}

static int netsec_netdev_set_features(struct net_device *ndev,
				      netdev_features_t features)
{
	struct netsec_priv *priv = netdev_priv(ndev);

	priv->rx_cksum_offload_flag = !!(features & NETIF_F_RXCSUM);

	return 0;
}

static void netsec_phy_adjust_link(struct net_device *ndev)
{
	struct netsec_priv *priv = netdev_priv(ndev);

	if (priv->actual_link_speed == ndev->phydev->speed &&
	    priv->actual_duplex == ndev->phydev->duplex)
		return;

	phy_print_status(ndev->phydev);

	netsec_stop_gmac(priv);
	netsec_start_gmac(priv);
}

static irqreturn_t netsec_irq_handler(int irq, void *dev_id)
{
	struct netsec_priv *priv = dev_id;
	u32 status = netsec_readl(priv, NETSEC_REG_TOP_STATUS) &
		     netsec_readl(priv, NETSEC_REG_TOP_INTEN);

	if (!status)
		return IRQ_NONE;

	if (status & (NETSEC_IRQ_TX | NETSEC_IRQ_RX)) {
		netsec_writel(priv, NETSEC_REG_INTEN_CLR,
			      status & (NETSEC_IRQ_TX | NETSEC_IRQ_RX));
		napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

static void netsec_reset_hardware(struct netsec_priv *priv)
{
	/* stop DMA engines */
	if (!netsec_readl(priv, NETSEC_REG_ADDR_DIS_CORE)) {
		netsec_writel(priv, NETSEC_REG_DMA_HM_CTRL,
			      NETSEC_DMA_CTRL_REG_STOP);
		netsec_writel(priv, NETSEC_REG_DMA_MH_CTRL,
			      NETSEC_DMA_CTRL_REG_STOP);

		while (netsec_readl(priv, NETSEC_REG_DMA_HM_CTRL) &
		       NETSEC_DMA_CTRL_REG_STOP)
			cpu_relax();

		while (netsec_readl(priv, NETSEC_REG_DMA_MH_CTRL) &
		       NETSEC_DMA_CTRL_REG_STOP)
			cpu_relax();
	}

	netsec_writel(priv, NETSEC_REG_SOFT_RST, NETSEC_SOFT_RST_REG_RESET);
	netsec_writel(priv, NETSEC_REG_SOFT_RST, NETSEC_SOFT_RST_REG_RUN);
	netsec_writel(priv, NETSEC_REG_COM_INIT, NETSEC_COM_INIT_REG_ALL);

	while (netsec_readl(priv, NETSEC_REG_COM_INIT) != 0)
		cpu_relax();
}

static int netsec_netdev_open(struct net_device *ndev)
{
	struct netsec_priv *priv = netdev_priv(ndev);
	int ret, n;

	pm_runtime_get_sync(priv->dev);

	netsec_reset_hardware(priv);

	for (n = 0; n <= NETSEC_RING_MAX; n++) {
		ret = netsec_alloc_desc_ring(priv, n);
		if (ret) {
			netif_err(priv, probe, priv->ndev,
				  "%s: alloc ring failed\n", __func__);
			goto err;
		}
	}

	ret = netsec_setup_rx_desc(priv, &priv->desc_ring[NETSEC_RING_RX]);
	if (ret) {
		netif_err(priv, probe, priv->ndev,
			  "%s: fail setup ring\n", __func__);
		goto err1;
	}

	ret = netsec_init_hardware(priv);
	if (ret) {
		netif_err(priv, probe, priv->ndev,
			  "%s: netsec_init_hardware fail %d\n", __func__, ret);
		goto err1;
	}

	ret = request_irq(priv->ndev->irq, netsec_irq_handler,
			  IRQF_SHARED, "netsec", priv);
	if (ret) {
		netif_err(priv, drv, priv->ndev, "request_irq failed\n");
		goto err1;
	}
	priv->irq_registered = true;

	ret = netsec_clean_rx_desc_ring(priv);
	if (ret) {
		netif_err(priv, drv, priv->ndev,
			  "%s: clean rx desc fail\n", __func__);
		goto err2;
	}

	ret = netsec_clean_tx_desc_ring(priv);
	if (ret) {
		netif_err(priv, drv, priv->ndev,
			  "%s: clean tx desc fail\n", __func__);
		goto err2;
	}

	netsec_ring_irq_clr(priv, NETSEC_RING_TX, NETSEC_IRQ_EMPTY);

	if (dev_of_node(priv->dev)) {
		if (!of_phy_connect(priv->ndev, priv->phy_np,
				    netsec_phy_adjust_link, 0,
				    priv->phy_interface)) {
			netif_err(priv, link, priv->ndev, "missing PHY\n");
			goto err2;
		}
	} else {
		ret = phy_connect_direct(priv->ndev, priv->phydev,
					 netsec_phy_adjust_link,
					 priv->phy_interface);
		if (ret) {
			netif_err(priv, link, priv->ndev,
				  "phy_connect_direct() failed (%d)\n", ret);
			goto err2;
		}
	}

	phy_start_aneg(ndev->phydev);

	netsec_ring_irq_disable(priv, NETSEC_RING_TX, NETSEC_IRQ_EMPTY);

	netsec_start_gmac(priv);
	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	netsec_writel(priv, NETSEC_REG_INTEN_SET,
		      NETSEC_IRQ_TX | NETSEC_IRQ_RX);

	return 0;

err2:
	pm_runtime_put_sync(priv->dev);
	free_irq(priv->ndev->irq, priv);
	priv->irq_registered = false;
err1:
	for (n = 0; n <= NETSEC_RING_MAX; n++)
		netsec_free_desc_ring(priv, &priv->desc_ring[n]);
err:
	pm_runtime_put_sync(priv->dev);

	return ret;
}

static int netsec_netdev_stop(struct net_device *ndev)
{
	struct netsec_priv *priv = netdev_priv(ndev);
	int n;

	phy_stop(ndev->phydev);
	phy_disconnect(ndev->phydev);

	netif_stop_queue(priv->ndev);
	napi_disable(&priv->napi);

	netsec_writel(priv, NETSEC_REG_INTEN_CLR, ~0);
	netsec_stop_gmac(priv);

	pm_runtime_put_sync(priv->dev);

	for (n = 0; n <= NETSEC_RING_MAX; n++)
		netsec_free_desc_ring(priv, &priv->desc_ring[n]);

	free_irq(priv->ndev->irq, priv);
	priv->irq_registered = false;

	return 0;
}

const struct net_device_ops netsec_netdev_ops = {
	.ndo_open		= netsec_netdev_open,
	.ndo_stop		= netsec_netdev_stop,
	.ndo_start_xmit		= netsec_netdev_start_xmit,
	.ndo_set_features	= netsec_netdev_set_features,
	.ndo_set_mac_address    = eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};
