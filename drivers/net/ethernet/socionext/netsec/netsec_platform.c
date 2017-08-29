/**
 * drivers/net/ethernet/socionext/netsec/netsec_platform.c
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

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/netdevice.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

#include "netsec.h"

#define NETSEC_F_NETSEC_VER_MAJOR_NUM(x) (x & 0xffff0000)

static int napi_weight = 64;
static u16 pause_time = 256;

static int netsec_of_probe(struct platform_device *pdev,
			   struct netsec_priv *priv)
{
	int clk_count, ret, i;

	priv->phy_np = of_parse_phandle(pdev->dev.of_node, "phy-handle", 0);
	if (!priv->phy_np) {
		dev_err(&pdev->dev, "missing required property 'phy-handle'\n");
		return -EINVAL;
	}

	/* we require named clocks if there is more than one */
	clk_count = of_property_count_strings(pdev->dev.of_node, "clock-names");
	if (clk_count > 1) {
		if (clk_count > ARRAY_SIZE(priv->clk)) {
			dev_err(&pdev->dev, "too many clocks specified (%d)\n",
				clk_count);
			return -EINVAL;
		}

		for (i = 0; i < clk_count; i++) {
			const char *clk_name;

			ret = of_property_read_string_index(pdev->dev.of_node,
							    "clock-names", i,
							    &clk_name);
			if (ret) {
				dev_err(&pdev->dev,
					"failed to parse 'clock-names'\n");
				return ret;
			}
			priv->clk[i] = devm_clk_get(&pdev->dev, clk_name);
			if (!strcmp(clk_name, "phy_refclk")) {
				priv->freq = clk_get_rate(priv->clk[i]);
				dev_dbg(&pdev->dev,
					"found PHY refclock #%d freq %u\n",
					i, priv->freq);
			}
		}
		priv->clock_count = clk_count;
	} else {
		priv->clk[0] = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(priv->clk)) {
			dev_err(&pdev->dev,
				"missing required property 'clocks'\n");
			return PTR_ERR(priv->clk);
		}
		priv->freq = clk_get_rate(priv->clk[0]);
		priv->clock_count = 1;
	}
	return 0;
}

static int netsec_acpi_probe(struct platform_device *pdev,
			     struct netsec_priv *priv, u32 *phy_addr)
{
	int ret;

	if (!IS_ENABLED(CONFIG_ACPI))
		return -ENODEV;

	ret = device_property_read_u32(&pdev->dev, "phy-channel", phy_addr);
	if (ret) {
		dev_err(&pdev->dev,
			"missing required property 'phy-channel'\n");
		return ret;
	}

	ret = device_property_read_u32(&pdev->dev,
				       "socionext,phy-clock-frequency",
				       &priv->freq);
	if (ret)
		dev_err(&pdev->dev,
			"missing required property 'socionext,phy-clock-frequency'\n");
	return ret;
}

static int netsec_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct netsec_priv *priv;
	struct resource *mmio_res, *eeprom_res, *irq_res;
	u8 *mac, macbuf[ETH_ALEN];
	u32 hw_ver, phy_addr;
	int ret;

	mmio_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mmio_res) {
		dev_err(&pdev->dev, "No MMIO resource found.\n");
		return -ENODEV;
	}

	eeprom_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!eeprom_res) {
		dev_info(&pdev->dev, "No EEPROM resource found.\n");
		return -ENODEV;
	}

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq_res) {
		dev_err(&pdev->dev, "No IRQ resource found.\n");
		return -ENODEV;
	}

	ndev = alloc_etherdev(sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	SET_NETDEV_DEV(ndev, &pdev->dev);
	platform_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;

	priv->msg_enable = NETIF_MSG_TX_ERR | NETIF_MSG_HW | NETIF_MSG_DRV |
			   NETIF_MSG_LINK | NETIF_MSG_PROBE;

	ndev->irq = irq_res->start;

	priv->phy_interface = device_get_phy_mode(&pdev->dev);
	if (priv->phy_interface < 0) {
		dev_err(&pdev->dev, "missing required property 'phy-mode'\n");
		ret = -ENODEV;
		goto free_ndev;
	}

	priv->ioaddr = devm_ioremap(&pdev->dev, mmio_res->start,
				    resource_size(mmio_res));
	if (!priv->ioaddr) {
		dev_err(&pdev->dev, "devm_ioremap() failed\n");
		ret = -ENXIO;
		goto free_ndev;
	}

	priv->eeprom_base = devm_memremap(&pdev->dev, eeprom_res->start,
					  resource_size(eeprom_res),
					  MEMREMAP_WT);
	if (!priv->eeprom_base) {
		dev_err(&pdev->dev, "devm_memremap() failed for EEPROM\n");
		ret = -ENXIO;
		goto free_ndev;
	}

	mac = device_get_mac_address(&pdev->dev, macbuf, sizeof(macbuf));
	if (mac)
		ether_addr_copy(ndev->dev_addr, mac);

	if (priv->eeprom_base &&
	    (!mac || !is_valid_ether_addr(ndev->dev_addr))) {
		const u8 *macp = priv->eeprom_base + NETSEC_EEPROM_MAC_ADDRESS;

		ndev->dev_addr[0] = macp[3];
		ndev->dev_addr[1] = macp[2];
		ndev->dev_addr[2] = macp[1];
		ndev->dev_addr[3] = macp[0];
		ndev->dev_addr[4] = macp[7];
		ndev->dev_addr[5] = macp[6];
	}

	if (!is_valid_ether_addr(ndev->dev_addr)) {
		dev_warn(&pdev->dev, "No MAC address found, using random\n");
		eth_hw_addr_random(ndev);
	}

	if (dev_of_node(&pdev->dev))
		ret = netsec_of_probe(pdev, priv);
	else
		ret = netsec_acpi_probe(pdev, priv, &phy_addr);
	if (ret)
		goto free_ndev;

	if (!priv->freq) {
		dev_err(&pdev->dev, "missing PHY reference clock frequency\n");
		ret = -ENODEV;
		goto free_ndev;
	}

	/* disable by default */
	priv->et_coalesce.rx_coalesce_usecs = 0;
	priv->et_coalesce.rx_max_coalesced_frames = 1;
	priv->et_coalesce.tx_coalesce_usecs = 0;
	priv->et_coalesce.tx_max_coalesced_frames = 1;

	ret = device_property_read_u32(&pdev->dev, "max-frame-size",
				       &ndev->max_mtu);
	if (ret < 0)
		ndev->max_mtu = ETH_DATA_LEN;

	priv->rx_pkt_buf_len = ndev->max_mtu + 22;
	priv->param.use_jumbo_pkt_flag = (ndev->max_mtu > ETH_DATA_LEN);

	pm_runtime_enable(&pdev->dev);
	/* runtime_pm coverage just for probe, open/close also cover it */
	pm_runtime_get_sync(&pdev->dev);

	hw_ver = netsec_readl(priv, NETSEC_REG_F_TAIKI_VER);
	/* this driver only supports F_TAIKI style NETSEC */
	if (NETSEC_F_NETSEC_VER_MAJOR_NUM(hw_ver) !=
	    NETSEC_F_NETSEC_VER_MAJOR_NUM(NETSEC_REG_NETSEC_VER_F_TAIKI)) {
		ret = -ENODEV;
		goto pm_disable;
	}

	dev_info(&pdev->dev, "hardware revision %d.%d\n",
		 hw_ver >> 16, hw_ver & 0xffff);

	priv->mac_mode.flow_start_th = NETSEC_FLOW_CONTROL_START_THRESHOLD;
	priv->mac_mode.flow_stop_th = NETSEC_FLOW_CONTROL_STOP_THRESHOLD;
	priv->mac_mode.pause_time = pause_time;
	priv->mac_mode.flow_ctrl_enable_flag = false;

	netif_napi_add(ndev, &priv->napi, netsec_netdev_napi_poll, napi_weight);

	ndev->netdev_ops = &netsec_netdev_ops;
	ndev->ethtool_ops = &netsec_ethtool_ops;
	ndev->features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
			 NETIF_F_TSO | NETIF_F_TSO6 | NETIF_F_GSO |
			 NETIF_F_HIGHDMA | NETIF_F_RXCSUM;
	ndev->hw_features = ndev->features;

	priv->rx_cksum_offload_flag = true;
	spin_lock_init(&priv->tx_queue_lock);

	ret = netsec_mii_register(priv);
	if (ret) {
		dev_err(&pdev->dev, "mii bus registration failed (%d)\n", ret);
		goto pm_disable;
	}

	if (!dev_of_node(&pdev->dev)) { /* ACPI */
		priv->phydev = get_phy_device(priv->mii_bus, phy_addr, false);
		if (IS_ERR(priv->phydev)) {
			dev_err(&pdev->dev, "get_phy_device() failed (%ld)\n",
				PTR_ERR(priv->phydev));
			ret = PTR_ERR(priv->phydev);
			goto unregister_mii;
		}

		ret = phy_device_register(priv->phydev);
		if (ret) {
			dev_err(&pdev->dev,
				"phy_device_register() failed (%d)\n", ret);
			phy_device_free(priv->phydev);
			goto unregister_mii;
		}
	}

	/* disable all other interrupt sources */
	netsec_writel(priv, NETSEC_REG_INTEN_CLR, ~0);
	netsec_writel(priv, NETSEC_REG_INTEN_SET,
		      NETSEC_IRQ_TX | NETSEC_IRQ_RX);

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64)))
		dev_warn(&pdev->dev, "Failed to enable 64-bit DMA\n");

	ret = register_netdev(ndev);
	if (ret) {
		netif_err(priv, probe, ndev, "register_netdev() failed\n");
		goto unregister_mii;
	}

	pm_runtime_put_sync_suspend(&pdev->dev);

	return 0;

unregister_mii:
	netsec_mii_unregister(priv);

pm_disable:
	pm_runtime_put_sync_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

free_ndev:
	free_netdev(ndev);

	dev_err(&pdev->dev, "init failed\n");

	return ret;
}

static int netsec_remove(struct platform_device *pdev)
{
	struct netsec_priv *priv = platform_get_drvdata(pdev);

	unregister_netdev(priv->ndev);
	if (!dev_of_node(&pdev->dev)) { /* ACPI */
		phy_device_remove(priv->phydev);
		phy_device_free(priv->phydev);
	}
	netsec_mii_unregister(priv);
	pm_runtime_disable(&pdev->dev);
	free_netdev(priv->ndev);

	return 0;
}

#ifdef CONFIG_PM
static int netsec_runtime_suspend(struct device *dev)
{
	struct netsec_priv *priv = dev_get_drvdata(dev);
	int n;

	netif_dbg(priv, drv, priv->ndev, "%s\n", __func__);

	if (priv->irq_registered)
		disable_irq(priv->ndev->irq);

	netsec_writel(priv, NETSEC_REG_CLK_EN, 0);

	for (n = priv->clock_count - 1; n >= 0; n--)
		clk_disable_unprepare(priv->clk[n]);

	return 0;
}

static int netsec_runtime_resume(struct device *dev)
{
	struct netsec_priv *priv = dev_get_drvdata(dev);
	int n;

	netif_dbg(priv, drv, priv->ndev, "%s\n", __func__);

	/* first let the clocks back on */

	for (n = 0; n < priv->clock_count; n++)
		clk_prepare_enable(priv->clk[n]);

	netsec_writel(priv, NETSEC_REG_CLK_EN, NETSEC_CLK_EN_REG_DOM_D |
					       NETSEC_CLK_EN_REG_DOM_C |
					       NETSEC_CLK_EN_REG_DOM_G);

	if (priv->irq_registered)
		enable_irq(priv->ndev->irq);

	return 0;
}

static int netsec_pm_suspend(struct device *dev)
{
	struct netsec_priv *priv = dev_get_drvdata(dev);

	netif_dbg(priv, drv, priv->ndev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return netsec_runtime_suspend(dev);
}

static int netsec_pm_resume(struct device *dev)
{
	struct netsec_priv *priv = dev_get_drvdata(dev);

	netif_dbg(priv, drv, priv->ndev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return netsec_runtime_resume(dev);
}
#endif

static const struct dev_pm_ops netsec_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(netsec_pm_suspend, netsec_pm_resume)
	SET_RUNTIME_PM_OPS(netsec_runtime_suspend, netsec_runtime_resume, NULL)
};

static const struct of_device_id netsec_dt_ids[] = {
	{ .compatible = "socionext,synquacer-netsec" },
	{ }
};
MODULE_DEVICE_TABLE(of, netsec_dt_ids);

#ifdef CONFIG_ACPI
static const struct acpi_device_id netsec_acpi_ids[] = {
	{ "SCX0001" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, netsec_acpi_ids);
#endif

static struct platform_driver netsec_driver = {
	.probe				= netsec_probe,
	.remove				= netsec_remove,
	.driver.name			= "netsec",
	.driver.of_match_table		= netsec_dt_ids,
	.driver.acpi_match_table	= ACPI_PTR(netsec_acpi_ids),
	.driver.pm			= &netsec_pm_ops,
};
module_platform_driver(netsec_driver);

MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_AUTHOR("Jassi Brar <jaswinder.singh@linaro.org>");
MODULE_AUTHOR("Ard Biesheuvel <ard.biesheuvel@linaro.org>");
MODULE_DESCRIPTION("NETSEC Ethernet driver");
MODULE_LICENSE("GPL");
