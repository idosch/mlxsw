/*
 * APM X-Gene PCIe ECAM fixup driver
 *
 * Copyright (c) 2016, Applied Micro Circuits Corporation
 * Author:
 *     Duc Dang <dhdang@apm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci-acpi.h>
#include <linux/platform_device.h>
#include <linux/pci-ecam.h>

#ifdef CONFIG_ACPI
#define RTDID                  0x160
#define ROOT_CAP_AND_CTRL      0x5C

/* PCIe IP version */
#define XGENE_PCIE_IP_VER_UNKN 0
#define XGENE_PCIE_IP_VER_1    1

#define APM_OEM_ID             "APM"
#define APM_XGENE_OEM_TABLE_ID "XGENE"
#define APM_XGENE_OEM_REV      0x00000002

struct xgene_pcie_acpi_root {
       void __iomem *csr_base;
       u32 version;
};

static acpi_status xgene_pcie_find_csr_base(struct acpi_resource *acpi_res,
                                           void *data)
{
       struct xgene_pcie_acpi_root *root = data;
       struct acpi_resource_fixed_memory32 *fixed32;

       if (acpi_res->type == ACPI_RESOURCE_TYPE_FIXED_MEMORY32) {
               fixed32 = &acpi_res->data.fixed_memory32;
               root->csr_base = ioremap(fixed32->address,
                                        fixed32->address_length);
               return AE_CTRL_TERMINATE;
       }

       return AE_OK;
}

static int xgene_pcie_ecam_init(struct pci_config_window *cfg)
{
       struct xgene_pcie_acpi_root *xgene_root;
       struct device *dev = cfg->parent;
       struct acpi_device *adev = to_acpi_device(dev);
       acpi_handle handle = acpi_device_handle(adev);

       xgene_root = devm_kzalloc(dev, sizeof(*xgene_root), GFP_KERNEL);
       if (!xgene_root)
               return -ENOMEM;

       acpi_walk_resources(handle, METHOD_NAME__CRS,
                           xgene_pcie_find_csr_base, xgene_root);

       if (!xgene_root->csr_base) {
               kfree(xgene_root);
               return -ENODEV;
       }

       xgene_root->version = XGENE_PCIE_IP_VER_1;

       cfg->priv = xgene_root;

       return 0;
}

/*
 * For Configuration request, RTDID register is used as Bus Number,
 * Device Number and Function number of the header fields.
 */
static void xgene_pcie_set_rtdid_reg(struct pci_bus *bus, uint devfn)
{
       struct pci_config_window *cfg = bus->sysdata;
       struct xgene_pcie_acpi_root *port = cfg->priv;
       unsigned int b, d, f;
       u32 rtdid_val = 0;

       b = bus->number;
       d = PCI_SLOT(devfn);
       f = PCI_FUNC(devfn);

       if (!pci_is_root_bus(bus))
               rtdid_val = (b << 8) | (d << 3) | f;

       writel(rtdid_val, port->csr_base + RTDID);
       /* read the register back to ensure flush */
       readl(port->csr_base + RTDID);
}

/*
 * X-Gene PCIe port uses BAR0-BAR1 of RC's configuration space as
 * the translation from PCI bus to native BUS.  Entire DDR region
 * is mapped into PCIe space using these registers, so it can be
 * reached by DMA from EP devices.  The BAR0/1 of bridge should be
 * hidden during enumeration to avoid the sizing and resource allocation
 * by PCIe core.
 */
static bool xgene_pcie_hide_rc_bars(struct pci_bus *bus, int offset)
{
       if (pci_is_root_bus(bus) && ((offset == PCI_BASE_ADDRESS_0) ||
                                    (offset == PCI_BASE_ADDRESS_1)))
               return true;

       return false;
}

void __iomem *xgene_pcie_ecam_map_bus(struct pci_bus *bus,
                                     unsigned int devfn, int where)
{
       struct pci_config_window *cfg = bus->sysdata;
       unsigned int busn = bus->number;
       void __iomem *base;

       if (busn < cfg->busr.start || busn > cfg->busr.end)
               return NULL;

       if ((pci_is_root_bus(bus) && devfn != 0) ||
           xgene_pcie_hide_rc_bars(bus, where))
               return NULL;

       xgene_pcie_set_rtdid_reg(bus, devfn);

       if (busn > cfg->busr.start)
               base = cfg->win + (1 << cfg->ops->bus_shift);
       else
               base = cfg->win;

       return base + where;
}

static int xgene_pcie_config_read32(struct pci_bus *bus, unsigned int devfn,
                                   int where, int size, u32 *val)
{
       struct pci_config_window *cfg = bus->sysdata;
       struct xgene_pcie_acpi_root *port = cfg->priv;

       if (pci_generic_config_read32(bus, devfn, where & ~0x3, 4, val) !=
           PCIBIOS_SUCCESSFUL)
               return PCIBIOS_DEVICE_NOT_FOUND;

       /*
       * The v1 controller has a bug in its Configuration Request
       * Retry Status (CRS) logic: when CRS is enabled and we read the
       * Vendor and Device ID of a non-existent device, the controller
       * fabricates return data of 0xFFFF0001 ("device exists but is not
       * ready") instead of 0xFFFFFFFF ("device does not exist").  This
       * causes the PCI core to retry the read until it times out.
       * Avoid this by not claiming to support CRS.
       */
       if (pci_is_root_bus(bus) && (port->version == XGENE_PCIE_IP_VER_1) &&
           ((where & ~0x3) == ROOT_CAP_AND_CTRL))
               *val &= ~(PCI_EXP_RTCAP_CRSVIS << 16);

       if (size <= 2)
               *val = (*val >> (8 * (where & 3))) & ((1 << (size * 8)) - 1);

       return PCIBIOS_SUCCESSFUL;
}

static struct pci_ecam_ops xgene_pcie_ecam_ops = {
       .bus_shift      = 16,
       .init           = xgene_pcie_ecam_init,
       .pci_ops        = {
               .map_bus        = xgene_pcie_ecam_map_bus,
               .read           = xgene_pcie_config_read32,
               .write          = pci_generic_config_write,
       }
};

DECLARE_ACPI_MCFG_FIXUP(&xgene_pcie_ecam_ops, APM_OEM_ID,
                       APM_XGENE_OEM_TABLE_ID, APM_XGENE_OEM_REV,
                       PCI_MCFG_DOMAIN_ANY, PCI_MCFG_BUS_ANY);
#endif
