/*
 * SMSC9303 switch driver
 *
 * Copyright (c) 2013 Stefan Roese <sr@denx.de>
 *
 * Based on this file from OpenWRT:
 * Lantiq PSB6970 (Tantos) Switch driver
 *
 * Copyright (c) 2009,2010 Team Embedded.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/switch.h>
#include <linux/phy.h>

#define SMSC9303_MAX_VLANS		16
#define SMSC9303_NUM_PORTS		3
#define SMSC9303_DEFAULT_PORT_CPU	0
#define SMSC9303_IS_CPU_PORT(x)		((x) == SMSC9303_DEFAULT_PORT_CPU)

#define PHYADDR(x)		((((x) >> 6) + 0x10) & 0x1f), (((x) >> 1) & 0x1f)

/* --- Identification --- */
#define SMSC9303_ID		0x0050
#define SMSC9303_ID_VAL		0x9303
#define SMSC9303_ID_MASK	0xffff

/* indirect access registers */
#define BYTE_TEST		0x64

#define SWITCH_CSR_DATA		0x1ac
#define SWITCH_CSR_CMD		0x1b0

#define SWITCH_CSR_CMD_BUSY	0x80000000
#define SWITCH_CSR_CMD_READ	0x40000000
#define SWITCH_CSR_CMD_BE_ALL	0x000f0000

#define SWE_VLAN_CMD		0x180b
#define SWE_VLAN_WR_DATA	0x180c
#define SWE_VLAN_RD_DATA	0x180e
#define SWE_VLAN_CMD_STS	0x1810

#define SWE_VLAN_CMD_VLAN_R	0x00000020
#define SWE_VLAN_CMD_VPID_SEL	0x00000010

#define SWE_VLAN_CMD_STS_BUSY	0x00000001

#define VLAN_UNTAG(port)	((0x1 << 12) << ((port) << 1))
#define VLAN_MEMBER(port)	((0x2 << 12) << ((port) << 1))

#define SWE_GLOBAL_INGRSS_CFG	0x1840
#define SWE_GLOBAL_INGRSS_CFG_VLAN_EN	0x00000001

#define TIMEOUT		100	/* msecs */

struct smsc9303_priv {
	struct switch_dev dev;
	struct phy_device *phy;
	u16 (*read) (struct phy_device* phydev, int reg);
	void (*write) (struct phy_device* phydev, int reg, u16 val);
	struct mutex reg_mutex;

	/* all fields below are cleared on reset */
	bool vlan;
	u16 vlan_id[SMSC9303_MAX_VLANS];
	u8 vlan_table[SMSC9303_MAX_VLANS];
	u8 vlan_tagged;
	u16 pvid[SMSC9303_NUM_PORTS];
};

#define to_smsc9303(_dev) container_of(_dev, struct smsc9303_priv, dev)

static u16 smsc9303_read(struct phy_device *phydev, int reg)
{
	return phydev->bus->read(phydev->bus, PHYADDR(reg));
}

static void smsc9303_write(struct phy_device *phydev, int reg, u16 val)
{
	phydev->bus->write(phydev->bus, PHYADDR(reg), val);
}

static u32 smsc9303_read32(struct phy_device *phydev, int reg)
{
	u32 val;

	mutex_lock(&phydev->bus->mdio_lock);
	val = smsc9303_read(phydev, reg);
	val |= (smsc9303_read(phydev, reg + 2) << 16) & 0xffff0000;
	mutex_unlock(&phydev->bus->mdio_lock);

	return val;
}

static void smsc9303_write32(struct phy_device *phydev, int reg, u32 val)
{
	mutex_lock(&phydev->bus->mdio_lock);
	smsc9303_write(phydev, reg, val & 0xffff);
	smsc9303_write(phydev, reg + 2, (val >> 16) & 0xffff);
	mutex_unlock(&phydev->bus->mdio_lock);
}

static int smsc9303_wait_idle(struct phy_device *dev)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(TIMEOUT);

	while (time_after(timeout, jiffies)) {
		if (!(smsc9303_read32(dev, SWITCH_CSR_CMD) & SWITCH_CSR_CMD_BUSY))
			return 0;
	}

	pr_err("Timed out waiting for idle (reg=0x%08x)!\n",
	       smsc9303_read32(dev, SWITCH_CSR_CMD));
	return -ETIMEDOUT;
}

static u32 smsc9303_read_indirect(struct phy_device *dev, int reg)
{
	volatile u32 tmp;

	smsc9303_wait_idle(dev);
	smsc9303_write32(dev, SWITCH_CSR_CMD,
			 SWITCH_CSR_CMD_BUSY | SWITCH_CSR_CMD_READ |
			 SWITCH_CSR_CMD_BE_ALL | reg);

	/* Flush the previous write by reading the BYTE_TEST register */
	tmp = smsc9303_read32(dev, BYTE_TEST);
	smsc9303_wait_idle(dev);

	return smsc9303_read32(dev, SWITCH_CSR_DATA);
}

static void smsc9303_write_indirect(struct phy_device *dev, int reg, u32 val)
{
	volatile u32 tmp;

	smsc9303_wait_idle(dev);
	smsc9303_write32(dev, SWITCH_CSR_DATA, val);
	smsc9303_write32(dev, SWITCH_CSR_CMD,
			 SWITCH_CSR_CMD_BUSY | SWITCH_CSR_CMD_BE_ALL | reg);

	/* Flush the previous write by reading the BYTE_TEST register */
	tmp = smsc9303_read32(dev, BYTE_TEST);
	smsc9303_wait_idle(dev);
}

/*
 * VLAN config access
 */
static int smsc9303_wait_idle_vlan(struct phy_device *dev)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(TIMEOUT);

	while (time_after(timeout, jiffies)) {
		if (!(smsc9303_read_indirect(dev, SWE_VLAN_CMD_STS) &
		       SWE_VLAN_CMD_STS_BUSY))
			return 0;
	}

	pr_err("Timed out waiting for idle (reg=0x%08x)!\n",
	       smsc9303_read_indirect(dev, SWE_VLAN_CMD_STS));
	return -ETIMEDOUT;
}

static u32 smsc9303_read_vlan(struct phy_device *dev, int pvid_select,
			      int vlan_port)
{
	u32 val = 0;

	if (pvid_select)
		val |= SWE_VLAN_CMD_VPID_SEL;

	smsc9303_wait_idle_vlan(dev);
	smsc9303_write_indirect(dev, SWE_VLAN_CMD,
				val | SWE_VLAN_CMD_VLAN_R | vlan_port);
	smsc9303_wait_idle_vlan(dev);

	return smsc9303_read_indirect(dev, SWE_VLAN_RD_DATA);
}

static void smsc9303_write_vlan(struct phy_device *dev, int pvid_select,
				int vlan_port, u32 data)
{
	u32 val = 0;

	if (pvid_select)
		val |= SWE_VLAN_CMD_VPID_SEL;

	smsc9303_wait_idle_vlan(dev);
	smsc9303_write_indirect(dev, SWE_VLAN_WR_DATA, data);
	smsc9303_write_indirect(dev, SWE_VLAN_CMD, val | vlan_port);
	smsc9303_wait_idle_vlan(dev);
}

static int smsc9303_set_vlan(struct switch_dev *dev,
			     const struct switch_attr *attr,
			     struct switch_val *val)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);

	priv->vlan = !!val->value.i;

	return 0;
}

static int smsc9303_get_vlan(struct switch_dev *dev,
			     const struct switch_attr *attr,
			     struct switch_val *val)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);

	val->value.i = priv->vlan;

	return 0;
}

static int smsc9303_set_pvid(struct switch_dev *dev, int port, int vlan)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);

	pr_devel("set_pvid port %d vlan %d\n", port, vlan);
	/* make sure no invalid PVIDs get set */
	if (vlan >= dev->vlans)
		return -EINVAL;

	priv->pvid[port] = vlan;

	return 0;
}

static int smsc9303_get_pvid(struct switch_dev *dev, int port, int *vlan)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);

	pr_devel("get_pvid port %d\n", port);
	*vlan = priv->pvid[port];

	return 0;
}

static int smsc9303_set_vid(struct switch_dev *dev,
			    const struct switch_attr *attr,
			    struct switch_val *val)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);

	pr_devel("set_vid port %d vid %d\n", val->port_vlan, val->value.i);
	priv->vlan_id[val->port_vlan] = val->value.i;

	return 0;
}

static int smsc9303_get_vid(struct switch_dev *dev,
			    const struct switch_attr *attr,
			    struct switch_val *val)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);

	pr_devel("get_vid port %d\n", val->port_vlan);
	val->value.i = priv->vlan_id[val->port_vlan];

	return 0;
}

static struct switch_attr smsc9303_globals[] = {
	{
		.type = SWITCH_TYPE_INT,
		.name = "enable_vlan",
		.description = "Enable VLAN mode",
		.set = smsc9303_set_vlan,
		.get = smsc9303_get_vlan,
		.max = 1
	},
};

static struct switch_attr smsc9303_port[] = {
};

static struct switch_attr smsc9303_vlan[] = {
	{
		.type = SWITCH_TYPE_INT,
		.name = "vid",
		.description = "VLAN ID (0-4094)",
		.set = smsc9303_set_vid,
		.get = smsc9303_get_vid,
		.max = 4094,
	 },
};

static int smsc9303_get_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);
	u8 ports = priv->vlan_table[val->port_vlan];
	int i;

	pr_devel("get_ports port_vlan %d\n", val->port_vlan);

	val->len = 0;
	for (i = 0; i < SMSC9303_NUM_PORTS; i++) {
		struct switch_port *p;

		if (!(ports & (1 << i)))
			continue;

		p = &val->value.ports[val->len++];
		p->id = i;
		if (priv->vlan_tagged & (1 << i))
			p->flags = (1 << SWITCH_PORT_FLAG_TAGGED);
		else
			p->flags = 0;
	}

	return 0;
}

static int smsc9303_set_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);
	u8 *vt = &priv->vlan_table[val->port_vlan];
	int i, j;

	pr_devel("set_ports port_vlan %d ports", val->port_vlan);

	*vt = 0;
	for (i = 0; i < val->len; i++) {
		struct switch_port *p = &val->value.ports[i];

		if (p->flags & (1 << SWITCH_PORT_FLAG_TAGGED))
			priv->vlan_tagged |= (1 << p->id);
		else {
			priv->vlan_tagged &= ~(1 << p->id);
			priv->pvid[p->id] = val->port_vlan;

			/* make sure that an untagged port does not
			 * appear in other vlans */
			for (j = 0; j < SMSC9303_MAX_VLANS; j++) {
				if (j == val->port_vlan)
					continue;
				priv->vlan_table[j] &= ~(1 << p->id);
			}
		}

		*vt |= 1 << p->id;
	}

	return 0;
}

#if 0 // test-only
void dump_vlan_regs(struct phy_device *pdev)
{
	int i;

	for (i=0; i < 3; i++)
		printk("*** %s (%d): vpid[%d]=%08x\n", __func__, __LINE__, i, smsc9303_read_vlan(pdev, 1, i)); // test-only
	for (i=0; i < 16; i++)
		printk("*** %s (%d): vlan[%d]=%08x\n", __func__, __LINE__, i, smsc9303_read_vlan(pdev, 0, i)); // test-only
}
#endif

static int smsc9303_hw_apply(struct switch_dev *dev)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);
	struct phy_device *pdev = priv->phy;
	int i, j;
	u32 val;

	pr_devel("hw_apply\n");
	mutex_lock(&priv->reg_mutex);

	if (priv->vlan) {
		/* into the vlan translation unit */
		for (j = 0; j < SMSC9303_MAX_VLANS; j++) {
			u8 vp = priv->vlan_table[j];

			if (vp) {
				u32 val = j;

				for (i = 0; i < SMSC9303_NUM_PORTS; i++) {
					if (vp & (1 << i)) {
						val |= VLAN_MEMBER(i);
						if (!(SMSC9303_IS_CPU_PORT(i)))
							val |= VLAN_UNTAG(i);
					}
				}
				smsc9303_write_vlan(pdev, 0, j, val);
			} else {
				/* clear VLAN Valid flag for unused vlans */
				smsc9303_write_vlan(pdev, 0, j, j);
			}
		}
	}

	/* update the default pvid's of each port */
	for (i = 0; i < SMSC9303_NUM_PORTS; i++) {
		int dvid = 1;

		if (priv->vlan)
			dvid = priv->vlan_id[priv->pvid[i]];

		if (!SMSC9303_IS_CPU_PORT(i))
			smsc9303_write_vlan(pdev, 1, i, dvid);
	}

	val = smsc9303_read_indirect(pdev, SWE_GLOBAL_INGRSS_CFG);
	/* Enable or disable VLAN engine */
	if (priv->vlan)
		val |= SWE_GLOBAL_INGRSS_CFG_VLAN_EN;
	else
		val &= ~SWE_GLOBAL_INGRSS_CFG_VLAN_EN;
	smsc9303_write_indirect(pdev, SWE_GLOBAL_INGRSS_CFG, val);

	mutex_unlock(&priv->reg_mutex);
//	dump_vlan_regs(pdev); // test-only

	return 0;
}

static int smsc9303_reset_switch(struct switch_dev *dev)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);
	int i;

	mutex_lock(&priv->reg_mutex);

	memset(&priv->vlan, 0, sizeof(struct smsc9303_priv) -
	       offsetof(struct smsc9303_priv, vlan));

	for (i = 0; i < SMSC9303_MAX_VLANS; i++)
		priv->vlan_id[i] = i;

	mutex_unlock(&priv->reg_mutex);

	return smsc9303_hw_apply(dev);
}

static int smsc9303_get_port_link(struct switch_dev *dev, int port,
				  struct switch_port_link *link)
{
	struct smsc9303_priv *priv = to_smsc9303(dev);
	struct phy_device *pdev = priv->phy;
	int status;

	memset(link, '\0', sizeof(*link));

	/* Special handling for the CPU port */
	if (SMSC9303_IS_CPU_PORT(port)) {
		link->speed = SPEED_100;
		link->duplex = DUPLEX_FULL;
		link->link = 1;
		return 0;
	}

	status = pdev->bus->read(pdev->bus, port, MII_BMSR);
	if (status < 0)
		return status;

	if ((status & BMSR_LSTATUS) == 0)
		link->link = 0;
	else
		link->link = 1;

	if (port == 0)
		status = smsc9303_read(pdev, 0x1c0);
	else
		status = pdev->bus->read(pdev->bus, port, MII_BMCR);
	if (status < 0)
		return status;

	if (status & BMCR_FULLDPLX)
		link->duplex = DUPLEX_FULL;
	else
		link->duplex = DUPLEX_HALF;

	if (status & BMCR_SPEED1000)
		link->speed = SPEED_1000;
	else if (status & BMCR_SPEED100)
		link->speed = SPEED_100;
	else
		link->speed = SPEED_10;

	return 0;
}

static const struct switch_dev_ops smsc9303_ops = {
	.attr_global = {
			.attr = smsc9303_globals,
			.n_attr = ARRAY_SIZE(smsc9303_globals),
			},
	.attr_port = {
		      .attr = smsc9303_port,
		      .n_attr = ARRAY_SIZE(smsc9303_port),
		      },
	.attr_vlan = {
		      .attr = smsc9303_vlan,
		      .n_attr = ARRAY_SIZE(smsc9303_vlan),
		      },
	.get_port_pvid = smsc9303_get_pvid,
	.set_port_pvid = smsc9303_set_pvid,
	.get_vlan_ports = smsc9303_get_ports,
	.set_vlan_ports = smsc9303_set_ports,
	.apply_config = smsc9303_hw_apply,
	.reset_switch = smsc9303_reset_switch,
	.get_port_link = smsc9303_get_port_link,
};

static int smsc9303_config_init(struct phy_device *pdev)
{
	struct smsc9303_priv *priv;
	struct switch_dev *swdev;
	int ret;

	priv = kzalloc(sizeof(struct smsc9303_priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->phy = pdev;

	if (pdev->addr == 0)
		printk(KERN_INFO "%s: smsc9303 switch driver attached.\n",
		       pdev->attached_dev->name);

	if (pdev->addr != 0) {
		kfree(priv);
		return 0;
	}

	pdev->supported = pdev->advertising = SUPPORTED_100baseT_Full;

	mutex_init(&priv->reg_mutex);
	priv->read = smsc9303_read;
	priv->write = smsc9303_write;

	pdev->priv = priv;

	swdev = &priv->dev;
	swdev->cpu_port = SMSC9303_DEFAULT_PORT_CPU;
	swdev->ops = &smsc9303_ops;

	swdev->name = "SMSC9303";
	swdev->vlans = SMSC9303_MAX_VLANS;
	swdev->ports = SMSC9303_NUM_PORTS;

	if ((ret = register_switch(&priv->dev, pdev->attached_dev)) < 0) {
		kfree(priv);
		goto done;
	}

	ret = smsc9303_reset_switch(&priv->dev);
	if (ret) {
		kfree(priv);
		goto done;
	}

done:
	return ret;
}

static int smsc9303_read_status(struct phy_device *phydev)
{
	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;
	phydev->link = 1;

	phydev->state = PHY_RUNNING;
	netif_carrier_on(phydev->attached_dev);
	phydev->adjust_link(phydev->attached_dev);

	return 0;
}

static int smsc9303_config_aneg(struct phy_device *phydev)
{
	return 0;
}

static int smsc9303_probe(struct phy_device *pdev)
{
	return 0;
}

static void smsc9303_remove(struct phy_device *pdev)
{
	struct smsc9303_priv *priv = pdev->priv;

	if (!priv)
		return;

	if (pdev->addr == 0)
		unregister_switch(&priv->dev);
	kfree(priv);
}

static int smsc9303_fixup(struct phy_device *dev)
{
	/*
	 * One dummy read access needed to reliably detect the switch
	 */
	smsc9303_read32(dev, BYTE_TEST);
	dev->phy_id = smsc9303_read32(dev, SMSC9303_ID);

	return 0;
}

static struct phy_driver smsc9303_driver = {
	.name = "SMSC9303",
	.phy_id = SMSC9303_ID_VAL << 16,
	.phy_id_mask = 0xffff0000,
	.features = PHY_BASIC_FEATURES,
	.probe = smsc9303_probe,
	.remove = smsc9303_remove,
	.config_init = &smsc9303_config_init,
	.config_aneg = &smsc9303_config_aneg,
	.read_status = &smsc9303_read_status,
	.driver = {
		.owner = THIS_MODULE
	},
};

int __init smsc9303_init(void)
{
	phy_register_fixup_for_id(PHY_ANY_ID, smsc9303_fixup);
	return phy_driver_register(&smsc9303_driver);
}

module_init(smsc9303_init);

void __exit smsc9303_exit(void)
{
	phy_driver_unregister(&smsc9303_driver);
}

module_exit(smsc9303_exit);

MODULE_DESCRIPTION("SMSC9303 Switch");
MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_LICENSE("GPL");
