/* drivers/battery/rt5033_fuelgauge.c
 * RT5033 Voltage Tracking Fuelgauge Driver
 *
 * Copyright (C) 2013
 * Author: Dongik Sin <dongik.sin@samsung.com>
 * Modified by Patrick Chang <patrick_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/battery/sec_fuelgauge.h>
#include <linux/battery/fuelgauge/rt5033_fuelgauge.h>
#include <linux/battery/fuelgauge/rt5033_fuelgauge_impl.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/math64.h>
#include <linux/compiler.h>
#define RT5033_FG_DEVICE_NAME "rt5033-fg"
#define ALIAS_NAME "rt5033-fuelgauge"

#define FG_DET_BAT_PRESENT 1
#define MINVAL(a, b) ((a <= b) ? a : b)

static int offset_li(int voltNR, int tempNR,
                    const data_point_t *mesh,
                    int volt, int temp);
static vg_comp_data_t vgcomp_li(int voltNR, int tempNR,
                         const data_point_t *mesh,
                         int volt, int temp);

static int rt5033_fg_get_offset(struct sec_fuelgauge_info *fuelgauge,
                         int state, int volt, int temp);

static vg_comp_data_t rt5033_fg_get_vgcomp(
                                    struct sec_fuelgauge_info* fuelgauge,
                                    int state, int volt, int temp);

static inline const data_point_t *get_mesh_data(
                                        int i, int j,
                                        const data_point_t *mesh, int xNR)
{
	return mesh + j * xNR + i;
}

typedef struct {
	int x,y;
	int order_x, order_y;
	int xNR, yNR;
	const data_point_t *mesh_src;
} submask_condition_t;

#define PRECISION_ENHANCE 5
/* n-order 2D interpolation */
static int offset_li(int xNR, int yNR, const data_point_t *mesh, int x, int y)
{
	long long retval = 0;
	int i, j, k;
	long long wM,wD;
	const data_point_t* cache;
	for (i = 0 ; i < xNR; ++i) {
		for (j = 0; j < yNR; ++j) {
			wM = wD = 1;
			cache = get_mesh_data(i, j, mesh, xNR);
			for (k = 0; k < xNR; ++k) {
				if (i != k) {
					wM *= (x - get_mesh_data(k, j, mesh, xNR)->x);
					wD *= (cache->x - get_mesh_data(k, j, mesh, xNR)->x);
				}
			}
			for (k = 0; k < yNR; ++k) {
				if (j != k) {
					wM *= (y - get_mesh_data(i, k, mesh, xNR)->y);
					wD *= (cache->y - get_mesh_data(i, k, mesh, xNR)->y);
				}
			}
			retval += div64_s64(((cache->z * wM) << PRECISION_ENHANCE), wD);
		}
	}
	return (int)((retval+ (1 << (PRECISION_ENHANCE-1))) >> PRECISION_ENHANCE);
}

static int get_sub_mesh(data_point_t *mesh_buffer,
			submask_condition_t *condition)
{
	int i, j, x, y;
	BUG_ON(condition->order_x > condition->xNR);
	BUG_ON(condition->order_y > condition->yNR);
	x = condition->x;
	y = condition->y;
	for (i = 0; i < condition->xNR; ++i) {
		if (get_mesh_data(i, 0, condition->mesh_src, condition->xNR)->x >= x)
			break;
	}
	for ( ; i >= 0 && i < condition->xNR; --i) {
		if (get_mesh_data(i, 0, condition->mesh_src, condition->xNR)->x <= x)
			break;
	}

	for (j = 0; j < condition->yNR; ++j) {
		if (get_mesh_data(0, j, condition->mesh_src, condition->xNR)->y >= y)
			break;
	}
	for ( ; j >= 0 && j < condition->yNR; --j) {
		if (get_mesh_data(0, j, condition->mesh_src, condition->xNR)->y <= y)
			break;
	}
	i -= ((condition->order_x - 1) / 2);
	j -= ((condition->order_y - 1) / 2);

	if (i <= 0)
		i = 0;
	if (j <= 0)
		j = 0;
	if ((i + condition->order_x) > condition->xNR)
		i = condition->xNR - condition->order_x;
	if ((j + condition->order_y) > condition->yNR)
		j = condition->yNR - condition->order_y;
	for (y = 0; y < condition->order_y; ++y) {
		for (x = 0; x < condition->order_x; ++x) {
			*(mesh_buffer + y * condition->order_x + x)
				= *get_mesh_data(i + x, j + y, condition->mesh_src, condition->xNR);
		}
	}
	return 0;
}

/* n-order 2D interpolation */
static vg_comp_data_t vgcomp_li(int xNR, int yNR, const data_point_t *mesh,
				int x, int y)
{
	long long retval[] = { 0, 0, 0, 0};
	vg_comp_data_t ret;
	int i, j, k;
	long long wM, wD;
	const data_point_t* cache;
	for (i = 0 ; i < xNR; ++i) {
		for (j = 0; j < yNR; ++j) {
			wM = wD = 1;
			cache = get_mesh_data(i, j, mesh, xNR);
			for (k = 0; k < xNR; ++k) {
				if (i != k) {
					wM *= (x - get_mesh_data(k, j, mesh, xNR)->x);
					wD *= (cache->x - get_mesh_data(k, j, mesh, xNR)->x);
				}
			}
			for (k = 0; k < yNR; ++k) {
				if (j != k) {
					wM *= (y - get_mesh_data(i, k, mesh, xNR)->y);
					wD *= (cache->y - get_mesh_data(i, k, mesh, xNR)->y);
				}
			}
			for (k = 0; k < ARRAY_SIZE(retval); ++k)
				retval[k]  +=  div64_s64((cache->vg_comp.data[k] * wM) << PRECISION_ENHANCE, wD);
		}
	}
	for (k = 0; k < ARRAY_SIZE(retval); ++k) {
		ret.data[k] = (int)((retval[k] + (1 << (PRECISION_ENHANCE - 1))) >> PRECISION_ENHANCE);
		if (unlikely(ret.data[k] < 0))
			ret.data[k]  = 0;
		else if (unlikely(ret.data[k] > 255))
			ret.data[k] = 255;
	}
	return ret;
}



static inline int rt5033_fg_read_device(struct i2c_client *client,
				      int reg, int bytes, void *dest)
{
	int ret;

	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(client, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	return ret;
}

static int32_t rt5033_fg_i2c_read_word(struct i2c_client *client,
                        uint8_t reg_addr)
{
	uint16_t data = 0;
	int ret;
	ret = rt5033_fg_read_device(client, reg_addr, 2, &data);
	if (ret < 0)
		return ret;
	else
		return (int32_t)be16_to_cpu(data);
}


static int32_t rt5033_fg_i2c_write_word(struct i2c_client *client,
                            uint8_t reg_addr,uint16_t data)
{
	int ret;

	data = cpu_to_be16(data);
	ret = i2c_smbus_write_i2c_block_data(client, reg_addr, 2, (uint8_t *)&data);
	return ret;
}

static unsigned int fg_get_vbat(struct i2c_client *client);
static unsigned int fg_get_avg_volt(struct i2c_client *client);
static unsigned int fg_get_ocv(struct i2c_client *client);

static void rt5033_pr_ver_info(struct i2c_client *client)
{
	dev_info(&client->dev, "RT5033 Fuel-Gauge Ver %s\n", FG_DRIVER_VER);
}
unsigned int fg_get_soc(struct i2c_client *client)
{
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	int ret;
	unsigned int volt, ocv;
	int temp;
	int offset = 0, offset_chg_dischg = 0;
	int crate;
	int idle;
	int chg_exist;
	vg_comp_data_t vgcomp;
	int soc_val;

	crate = rt5033_fg_i2c_read_word(client, RT5033_CRATE) >> 8;
	idle = (crate >= fuelgauge->info.crate_idle_thres) ? 1 : 0;
	ret = rt5033_fg_i2c_read_word(client, RT5033_CONFIG_MSB);
	/* disable real time battery presence detection &
	 * sleep mode & battery type detection */
	ret &= ~(0x0B01);
	/* enable battery presence and set battery type*/
	if (fuelgauge->info.battery_type >= 4350)
		ret |= 0x0401;
	else
		ret |= 0x0400;
	/* Enable battery real-time detection if TA/USB is exist*/
	chg_exist = fuelgauge->info.flag_full_charge |
		fuelgauge->info.flag_chg_status;
	if (chg_exist)
		ret |= 0x0800;
	rt5033_fg_i2c_write_word(client, RT5033_CONFIG_MSB, ret);

	// read rt5033 fg registers for debugging.
	ret = fg_get_vbat(client);
	pr_info("%s : FG_VBAT = %d\n", __func__, ret);
	ret = rt5033_fg_i2c_read_word(client, RT5033_FG_IRQ_CTRL);
	pr_info("%s : FG_IRQ_CTL_FLAG = 0x%04x\n", __func__, ret);
	ret = rt5033_fg_i2c_read_word(client, RT5033_VOLT_ALRT_TH);
	pr_info("%s : FG_VMIN_SMIN_FLAG = 0x%04x\n", __func__, ret);
	ret = rt5033_fg_i2c_read_word(client, RT5033_CRATE);
	pr_info("%s : FG_CRATE_DEVICEID = 0x%04x\n", __func__, ret);

	/* clear ri */
	ret = rt5033_fg_i2c_read_word(client, RT5033_CONFIG_MSB);

	// read FG_CONFIG  for debugging.
	pr_info("%s : FG_CONFIG = 0x%04x\n", __func__, ret);

	if (ret & 0x0040) {
		ret &= ~(0x0040);
		rt5033_fg_i2c_write_word(client, RT5033_CONFIG_MSB, ret);
	}

	/* cvs */
	temp = fuelgauge->info.temperature / 10;
	if (temp < R5_TEMP)
		rt5033_fg_i2c_write_word(client, RT5033_MFA_MSB, MFA_CMD_R5_LOW_TEMP);
	else
		rt5033_fg_i2c_write_word(client, RT5033_MFA_MSB, MFA_CMD_R5_DEFAULT);

	rt5033_fg_i2c_write_word(client, RT5033_MFA_MSB, 0x8085);
	ret = rt5033_fg_i2c_read_word(client, RT5033_MFA_MSB);
	pr_info("%s : FG_CVS = 0x%04x\n", __func__, ret);

	ocv = fg_get_ocv(client);
	volt = fg_get_vbat(client);

	/* report FG_OCV & FG_TEMP for debugging. */
	pr_info("%s : FG_OCV = %d, FG_TEMP = %d, FG_VBAT = %d\n",
			__func__, ocv, temp, volt);

	vgcomp = rt5033_fg_get_vgcomp(fuelgauge,
				      idle ? VGCOMP_IDLE : VGCOMP_NORMAL,
				      ocv, fuelgauge->info.temperature);
	ret = (vgcomp.data[0] << 8) | vgcomp.data[1];
	rt5033_fg_i2c_write_word(client, RT5033_VGCOMP1, ret);

	ret = (vgcomp.data[2] << 8) | vgcomp.data[3];
	rt5033_fg_i2c_write_word(client, RT5033_VGCOMP3, ret);
	pr_info("%s : VGCOMP(%s) = 0x%04x, 0x%04x\n", __func__,
			idle ? "idle" : "normal",
			(vgcomp.data[0] << 8) | vgcomp.data[1],
			(vgcomp.data[2] << 8) | vgcomp.data[3]);
	/* report VGCOMP1~4 for debugging. */
	ret = rt5033_fg_i2c_read_word(client, RT5033_VGCOMP1);
	pr_info("%s : FG_VGCOMP1_2 = 0x%04x\n", __func__, ret);
	ret = rt5033_fg_i2c_read_word(client, RT5033_VGCOMP3);
	pr_info("%s : FG_VGCOMP3_4 = 0x%04x\n", __func__, ret);


	/* read soc */
	ret = rt5033_fg_i2c_read_word(client, RT5033_SOC_MSB);
	if (ret < 0) {
		pr_err("%s: read soc reg fail", __func__);
		soc_val = 500;
	} else {
		soc_val = ((ret) >> 8) * 10 + (ret & 0x00ff) * 10 / 256;
	}
	/* chg/dischg offset */
	offset_chg_dischg = rt5033_fg_get_offset(fuelgauge, chg_exist ?
				       OFFSET_CHARGING : OFFSET_DISCHARGING,
				      soc_val, fuelgauge->info.temperature);

	offset = rt5033_fg_get_offset(fuelgauge, OFFSET_LOW_POWER,
					soc_val, fuelgauge->info.temperature);
	soc_val = soc_val + offset_chg_dischg + offset;
	/* report FG_SOC_RAW for debugging. */
	pr_info("%s : FG_SOC_RAW = 0x%04x\n", __func__, ret);
	pr_info("%s : CHG/DISCHG OFFSET  = %d\n", __func__, offset_chg_dischg);
	pr_info("%s : FG_LOW_POWER_OFFS = %d\n", __func__, offset);

	if (soc_val < 0) {
		soc_val = 0;
	}

	fuelgauge->info.batt_soc = soc_val;
	/* report FG_SOC for debugging. */
	pr_info("%s : FG_SOC = %d\n", __func__, soc_val);

	return soc_val;
}

static unsigned int fg_get_ocv(struct i2c_client *client)
{
	int ret;
	unsigned int ocv;// = 3500; /*3500 means 3500mV*/
	ret = rt5033_fg_i2c_read_word(client, RT5033_OCV_MSB);
	if (ret<0) {
		pr_err("%s: read soc reg fail", __func__);
		ocv = 4000;
	} else {
		ocv = (ret&0xfff0)>>4;
		ocv = ocv * 125 / 100;
	}
	return ocv;
}

static unsigned int fg_get_vbat(struct i2c_client *client)
{
	int ret;
	unsigned int vbat;/* = 3500; 3500 means 3500mV*/
	ret = rt5033_fg_i2c_read_word(client, RT5033_VBAT_MSB);
	if (ret<0) {
		pr_err("%s: read vbat fail", __func__);
		vbat = 4000;
	} else {
		vbat = (ret&0xfff0)>>4;
		vbat = vbat * 125 / 100;
	}
	return vbat;
}

static unsigned int fg_get_avg_volt(struct i2c_client *client)
{
	int ret;
	unsigned int avg_volt;/* = 3500; 3500 means 3500mV*/
	ret = rt5033_fg_i2c_read_word(client, RT5033_AVG_VOLT_MSB);
	if (ret<0) {
		pr_err("%s: read vbat fail", __func__);
		avg_volt = 4000;
	} else {
		avg_volt = (ret&0xfff0)>>4;
		avg_volt = avg_volt * 125 / 100;
	}
	return avg_volt;
}

static unsigned int fg_get_device_id(struct i2c_client *client)
{
	int ret;
	ret = rt5033_fg_i2c_read_word(client, RT5033_CRATE);
	ret &= 0x00ff;
	return ret;
}

static bool rt5033_fg_get_batt_present(struct i2c_client *client)
{
    int ret = rt5033_fg_i2c_read_word(client,RT5033_CONFIG_MSB);
    if (ret<0)
        return false;
    return (ret & 0x02)? true : false;
}

static vg_comp_data_t rt5033_fg_get_vgcomp(
                                    struct sec_fuelgauge_info* fuelgauge,
                                    int state, int volt, int temp)
{
    const int ip_x = fuelgauge->info.vg_comp_interpolation_order[0];
    const int ip_y = fuelgauge->info.vg_comp_interpolation_order[1];
    data_point_t sub_mesh[ip_x * ip_y];
    int xNR, yNR;
    const vg_comp_data_t default_vgcomp = {
        .data = { 0x32, 0x32, 0x32, 0x32, },
    };
    vg_comp_data_t retval;
    vg_comp_table_t *vgcomp_table = NULL;
    submask_condition_t condition = {
        .x = volt,
        .y = temp,
    };
    BUG_ON(state >= VGCOMP_NR);
    mutex_lock(&fuelgauge->info.param_lock);
    vgcomp_table = fuelgauge->info.vg_comp + state;
    xNR = vgcomp_table->voltNR;
    yNR = vgcomp_table->tempNR;
    dev_dbg(&fuelgauge->client->dev,
            "%s-%d: state = %d, xNR = %d, yNR = %d\n", __func__,
                __LINE__, state, xNR, yNR);
    if (xNR == 0 || yNR == 0) {
        mutex_unlock(&fuelgauge->info.param_lock);
        return default_vgcomp;
    }
    condition.order_x = MINVAL(ip_x, xNR);
    condition.order_y = MINVAL(ip_y, yNR);
    condition.xNR = xNR;
    condition.yNR = yNR;
    condition.mesh_src = vgcomp_table->vg_comp_data;
    dev_dbg(&fuelgauge->client->dev, "%s-%d: order_x = %d, order_y = %d\n",
            __func__, __LINE__, condition.order_x, condition.order_y);
    get_sub_mesh(sub_mesh, &condition);
    yNR = condition.order_x * condition.order_y;
    retval = vgcomp_li(condition.order_x, condition.order_y, sub_mesh,
                        volt, temp);
    mutex_unlock(&fuelgauge->info.param_lock);
    dev_dbg(&fuelgauge->client->dev, "%d %d %d %d\n", retval.data[0],
            retval.data[1], retval.data[2], retval.data[3]);
    return retval;
}

#define OFFSET_INTERPOLATION_ORDER_X 2
#define OFFSET_INTERPOLATION_ORDER_Y 2

static int rt5033_fg_get_offset(struct sec_fuelgauge_info* fuelgauge,
                         int state, int volt, int temp)
{
    const int ip_x = fuelgauge->info.offset_interpolation_order[0];
    const int ip_y = fuelgauge->info.offset_interpolation_order[1];
    data_point_t sub_mesh[ip_x * ip_y];
    int xNR, yNR;
    int offset;
    soc_offset_table_t *offset_table = NULL;
    submask_condition_t condition = {
        .x = volt,
        .y = temp,
    };
    BUG_ON(state >= OFFSET_NR);
    mutex_lock(&fuelgauge->info.param_lock);
    offset_table = fuelgauge->info.soc_offset + state;
    xNR = offset_table->soc_voltNR;
    yNR = offset_table->tempNR;
    dev_dbg(&fuelgauge->client->dev,
            "%s-%d: state = %d, xNR = %d, yNR = %d\n", __func__,
                __LINE__, state, xNR, yNR);
    if (xNR == 0 || yNR == 0) {
        mutex_unlock(&fuelgauge->info.param_lock);
        return 0;
    }
    condition.order_x = MINVAL(ip_x, xNR);
    condition.order_y = MINVAL(ip_y, yNR);
    condition.xNR = xNR;
    condition.yNR = yNR;
    condition.mesh_src = offset_table->soc_offset_data;
    get_sub_mesh(sub_mesh, &condition);
    offset = offset_li(condition.order_x, condition.order_y, sub_mesh,
                        volt, temp);
    mutex_unlock(&fuelgauge->info.param_lock);
    return offset;
}


static bool rt5033_fg_init(struct i2c_client *client)
{
	int ret;
	int ta_exist;
	union power_supply_propval value;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	board_fuelgauge_init(fuelgauge);
	/* disable hibernate mode */
	ret = rt5033_fg_i2c_read_word(client, RT5033_CONFIG_MSB);
	if (ret < 0) {
	    pr_err("rt5033_fg_init : failed to do i2c read(%d)\n", ret);
	    return false;
	}
	/* Disable SHDN */
	ret &= (~0x0080);
	rt5033_fg_i2c_write_word(client, RT5033_CONFIG_MSB, ret);

	/* disable hibernate mode */
	if (ret & 0x8000) {
		ret = rt5033_fg_i2c_write_word(client, RT5033_MFA_MSB, MFA_CMD_EXIT_HIB);
		pr_err("rt5033_fg_init : failed to do i2c write(%d)\n", ret);
		return false;
	}
	fg_get_device_id(client);
	value.intval = POWER_SUPPLY_HEALTH_UNKNOWN;
	psy_do_property("rt5033-charger", get,
			POWER_SUPPLY_PROP_HEALTH, value);

	ta_exist = (value.intval == POWER_SUPPLY_HEALTH_GOOD) |
                fuelgauge->is_charging;
    ret = rt5033_fg_i2c_read_word(client, RT5033_FG_IRQ_CTRL);
    if (ret < 0) {
	    printk("rt5033_fg_init : failed to do i2c read(%d)\n", ret);
	    return false;
	}
	if ((ret & 0x04)!=0 && ta_exist) {
		ret = rt5033_fg_i2c_read_word(client, RT5033_FG_IRQ_CTRL);
		ret &= (~0x04);
		rt5033_fg_i2c_write_word(client, RT5033_FG_IRQ_CTRL, ret);
		msleep(1000);
		/* send Quick Sensing command to 5033FG */
        rt5033_fg_i2c_write_word(client, 0x06, 0x8000);
	}
	return true;
}



enum comp_offset_type {
    NORMAL_VG_COMP = 0,
    IDLE_VG_COMP,
    SOC_OFFSET_CHARGING,
    SOC_OFFSET_DISCHARGING,
    SOC_OFFSET_SPECIAL,
    SOC_OFFSET_LOW_POWER,

};

static void new_vgcomp_soc_offset_data(int type,
				       struct sec_fuelgauge_info *fuelgauge, int size_x, int size_y)
{
	struct device *dev = &fuelgauge->client->dev;
	int index;

	switch (type) {
	case NORMAL_VG_COMP:
	case IDLE_VG_COMP:
		index = type - NORMAL_VG_COMP;
		if (fuelgauge->info.vg_comp[index].vg_comp_data) {
			devm_kfree(dev, fuelgauge->info.vg_comp[index].vg_comp_data);
			fuelgauge->info.vg_comp[index].vg_comp_data = NULL;
		}
		if (size_x != 0 && size_y != 0)
			fuelgauge->info.vg_comp[index].vg_comp_data =
				devm_kzalloc(dev, size_x * size_y * sizeof(data_point_t),
					     GFP_KERNEL);
		if (fuelgauge->info.vg_comp[index].vg_comp_data) {
			fuelgauge->info.vg_comp[index].voltNR = size_x;
			fuelgauge->info.vg_comp[index].tempNR = size_y;

		} else {
			fuelgauge->info.vg_comp[index].voltNR = 0;
			fuelgauge->info.vg_comp[index].tempNR = 0;
		}
		break;
	case SOC_OFFSET_CHARGING:
	case SOC_OFFSET_DISCHARGING:
	case SOC_OFFSET_SPECIAL:
	case SOC_OFFSET_LOW_POWER:
		index = type - SOC_OFFSET_CHARGING;
		if (fuelgauge->info.soc_offset[index].soc_offset_data) {
			devm_kfree(dev, fuelgauge->info.soc_offset[index].soc_offset_data);
			fuelgauge->info.soc_offset[index].soc_offset_data = NULL;
		}
		if (size_x != 0 && size_y != 0)
			fuelgauge->info.soc_offset[index].soc_offset_data =
				devm_kzalloc(dev, size_x * size_y * sizeof(data_point_t),
					     GFP_KERNEL);
		if (fuelgauge->info.soc_offset[index].soc_offset_data) {
			fuelgauge->info.soc_offset[index].soc_voltNR = size_x;
			fuelgauge->info.soc_offset[index].tempNR = size_y;

		} else {
			fuelgauge->info.soc_offset[index].soc_voltNR = 0;
			fuelgauge->info.soc_offset[index].tempNR = 0;
		}
		break;
	default:
		BUG();
	}
}

#ifdef CONFIG_OF
static int get_battery_id(struct sec_fuelgauge_info *fuelgauge)
{
    // rt5033fg does not support this function
    return 0;
}
#define PROPERTY_NAME_SIZE 128

#define PINFO(format, args...) \
	printk(KERN_INFO "%s() line-%d: " format, \
		__func__, __LINE__, ## args)

typedef struct {
    int data[3];
} dt_offset_params_t;

#define DECL_PARAM_PROP(_id, _name) {.id = _id, .name = _name,}
static struct {
	int id;
	const char *name;
} param_props[] = {
#ifdef CONFIG_MACH_ROSSA_CTC
	DECL_PARAM_PROP(NORMAL_VG_COMP, "vg_comp_normal_sub"),
	DECL_PARAM_PROP(IDLE_VG_COMP, "vg_comp_idle_sub"),
#else
	DECL_PARAM_PROP(NORMAL_VG_COMP, "vg_comp_normal"),
	DECL_PARAM_PROP(IDLE_VG_COMP, "vg_comp_idle"),
#endif
	DECL_PARAM_PROP(SOC_OFFSET_CHARGING, "offset_charging"),
	DECL_PARAM_PROP(SOC_OFFSET_DISCHARGING, "offset_discharging"),
	DECL_PARAM_PROP(SOC_OFFSET_SPECIAL, "offset_special"),
	DECL_PARAM_PROP(SOC_OFFSET_LOW_POWER, "offset_low_power"),
};

static int rt5033_fg_parse_dt(struct sec_fuelgauge_info *fuelgauge)
{
	struct device *dev = &fuelgauge->client->dev;
	struct device_node *np = dev->of_node;
	char prop_name[PROPERTY_NAME_SIZE];
	int battery_id = -1;
	int sizes[2];
	int ret;
	int i, j, index;
	dt_offset_params_t *offset_params;

	BUG_ON(dev == 0);
	BUG_ON(np == 0);
	np = of_find_node_by_name(of_node_get(np), "battery_params");
	if (np == NULL) {
		PINFO("Cannot find child node \"battery_params\"\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "battery,id", &battery_id) < 0)
		PINFO("not battery,id property\n");

	if (battery_id == -1)
		battery_id = get_battery_id(fuelgauge);
	PINFO("battery id = %d\n", battery_id);
	snprintf(prop_name, PROPERTY_NAME_SIZE,
		 "battery%d,vg_comp_interpolation_order", battery_id);
	ret = of_property_read_u32_array(np, prop_name,
					 fuelgauge->info.vg_comp_interpolation_order, 2);
	if (ret < 0)
		fuelgauge->info.vg_comp_interpolation_order[0] =
			fuelgauge->info.vg_comp_interpolation_order[1] = 2;
	PINFO("vg_comp interpolation order = <%d %d>\n",
	      fuelgauge->info.vg_comp_interpolation_order[0],
	      fuelgauge->info.vg_comp_interpolation_order[1]);

	for (i = NORMAL_VG_COMP; i <= IDLE_VG_COMP; ++i) {
		index = i - NORMAL_VG_COMP;
		snprintf(prop_name, PROPERTY_NAME_SIZE,
			 "battery%d,%s_size", battery_id, param_props[i].name);
		sizes[0] = sizes[1] = 0;
		ret = of_property_read_u32_array(np, prop_name, sizes, 2);
		if (ret < 0) {
			PINFO("Can get prop %s (%d)\n", prop_name, ret);
		}
		PINFO("%s = <%d %d>\n", prop_name, sizes[0], sizes[1]);
		new_vgcomp_soc_offset_data(i, fuelgauge, sizes[0], sizes[1]);
		if (fuelgauge->info.vg_comp[index].vg_comp_data) {

			snprintf(prop_name, PROPERTY_NAME_SIZE,
				 "battery%d,%s_data", battery_id, param_props[i].name);

			of_property_read_u32_array(np, prop_name,
						   (u32 *)fuelgauge->info.vg_comp[index].vg_comp_data,
						   sizes[0] * sizes[1] * 6);

			for (j = 0; j < sizes[0] * sizes[1]; ++j) {
				PINFO("<%d %d 0x%x 0x%x 0x%x 0x%x>\n",
				      fuelgauge->info.vg_comp[index].vg_comp_data[j].x,
				      fuelgauge->info.vg_comp[index].vg_comp_data[j].y,
				      fuelgauge->info.vg_comp[index].vg_comp_data[j].data[0],
				      fuelgauge->info.vg_comp[index].vg_comp_data[j].data[1],
				      fuelgauge->info.vg_comp[index].vg_comp_data[j].data[2],
				      fuelgauge->info.vg_comp[index].vg_comp_data[j].data[3]);
			}
		}
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE,
		 "battery%d,offset_interpolation_order", battery_id);
	ret = of_property_read_u32_array(np, prop_name,
					 fuelgauge->info.offset_interpolation_order, 2);
	if (ret < 0)
		fuelgauge->info.offset_interpolation_order[0] =
			fuelgauge->info.offset_interpolation_order[1] = 2;
	PINFO("offset interpolation order = <%d %d>\n",
	      fuelgauge->info.offset_interpolation_order[0],
	      fuelgauge->info.offset_interpolation_order[1]);

	for (i = SOC_OFFSET_CHARGING; i <= SOC_OFFSET_LOW_POWER; ++i) {
		index = i - SOC_OFFSET_CHARGING;
		snprintf(prop_name, PROPERTY_NAME_SIZE,
			 "battery%d,%s_size", battery_id, param_props[i].name);
		sizes[0] = sizes[1] = 0;
		ret = of_property_read_u32_array(np, prop_name, sizes, 2);
		if (ret < 0) {
			PINFO("Can get prop %s (%d)\n", prop_name, ret);
		}
		PINFO("%s = <%d %d>\n", prop_name, sizes[0], sizes[1]);
		new_vgcomp_soc_offset_data(i, fuelgauge, sizes[0], sizes[1]);
		if (fuelgauge->info.soc_offset[index].soc_offset_data) {
			offset_params = devm_kzalloc(dev,
						     sizes[0] * sizes[1] *
						     sizeof(dt_offset_params_t),
						     GFP_KERNEL);
			snprintf(prop_name, PROPERTY_NAME_SIZE,
				 "battery%d,%s_data", battery_id, param_props[i].name);

			of_property_read_u32_array(np, prop_name,
						   (u32 *)offset_params, sizes[0] * sizes[1] * 3);
			for (j = 0; j < sizes[0] * sizes[1]; ++j) {
				PINFO("<%d %d %d>\n", offset_params[j].data[0],
				      offset_params[j].data[1], offset_params[j].data[2]);
				fuelgauge->info.soc_offset[index].soc_offset_data[j].x =
					offset_params[j].data[0];
				fuelgauge->info.soc_offset[index].soc_offset_data[j].y =
					offset_params[j].data[1];
				fuelgauge->info.soc_offset[index].soc_offset_data[j].offset =
					offset_params[j].data[2];
			}
			devm_kfree(dev, offset_params);
		}
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE,
		 "battery%d,crate_idle_thres", battery_id);
	ret = of_property_read_u32_array(np, prop_name,
					 &fuelgauge->info.crate_idle_thres, 1);
	if (ret < 0)
		fuelgauge->info.crate_idle_thres = 256; /* disable*/
	PINFO("crate idle threshold = <%d>\n",
	      fuelgauge->info.crate_idle_thres);

	snprintf(prop_name, PROPERTY_NAME_SIZE,
		 "battery%d,battery_type", battery_id);
	ret = of_property_read_u32_array(np, prop_name,
					 &fuelgauge->info.battery_type, 1);
	if (ret < 0)
		fuelgauge->info.battery_type = 4350; /* default use 4350mV*/
	PINFO("battery_type = <%d>\n",
	      fuelgauge->info.battery_type);
	return 0;
}
#else
static int rt5033_fg_parse_dt(struct sec_fuelgauge_info *fuelgauge)
{
    return 0;
}
#endif

#ifdef CONFIG_DEBUG_FS

static int dentry_id_to_comp_offset_type[] = {
	[RT5033FG_OFFSET_CHARGING_SIZE] = SOC_OFFSET_CHARGING,
	[RT5033FG_OFFSET_CHARGING_DATA] = SOC_OFFSET_CHARGING,
	[RT5033FG_OFFSET_DISCHARGING_SIZE] = SOC_OFFSET_DISCHARGING,
	[RT5033FG_OFFSET_DISCHARGING_DATA] = SOC_OFFSET_DISCHARGING,
	[RT5033FG_OFFSET_SPECIAL_SIZE] = SOC_OFFSET_SPECIAL,
	[RT5033FG_OFFSET_SPECIAL_DATA] = SOC_OFFSET_SPECIAL,
	[RT5033FG_OFFSET_LOW_POWER_SIZE] = SOC_OFFSET_LOW_POWER,
	[RT5033FG_OFFSET_LOW_POWER_DATA] = SOC_OFFSET_LOW_POWER,
	[RT5033FG_VGCOMP_NORMAL_SIZE] = NORMAL_VG_COMP,
	[RT5033FG_VGCOMP_NORMAL_DATA] = NORMAL_VG_COMP,
	[RT5033FG_VGCOMP_IDLE_SIZE] = IDLE_VG_COMP,
	[RT5033FG_VGCOMP_IDLE_DATA] = IDLE_VG_COMP,
	[RT5033FG_PARAM_LOCK] = -1, /* dummy */
	[RT5033FG_VGCOMP_IP_ORDER] = -1, /* dummy */
	[RT5033FG_OFFSET_IP_ORDER] = -1, /* dummy */
	[RT5033FG_CRATE_IDLE_THRES] = -1, /* dummy */
};

static int get_parameters(char *buf, long int *param, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");

	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token != NULL) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (strict_strtol(token, base, &param[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
		} else
			return -EINVAL;
	}
	return 0;
}

typedef struct {
    struct sec_fuelgauge_info *fuelgague;
    int id;
    int counter;
} rt5033_dbg_private_data_t;


#define MAX_PARAMS 6


static int rt5033_fg_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t rt5033_fg_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
    rt5033_dbg_private_data_t *prv_data = filp->private_data;
    rt5033_dbg_private_data_t *prv_data_header;
    struct sec_fuelgauge_info *fuelgauge = prv_data->fuelgague;
    data_point_t *data;
	char lbuf[128];
	int rc;
	int index;
	int comp_offset_type;
	long int param[MAX_PARAMS];
	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;
	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;
	lbuf[cnt] = '\0';
	comp_offset_type = dentry_id_to_comp_offset_type[prv_data->id];
	prv_data_header = prv_data - prv_data->id;
	switch(prv_data->id) {
	case RT5033FG_OFFSET_CHARGING_SIZE:
	case RT5033FG_OFFSET_DISCHARGING_SIZE:
	case RT5033FG_OFFSET_SPECIAL_SIZE:
	case RT5033FG_OFFSET_LOW_POWER_SIZE:
	case RT5033FG_VGCOMP_NORMAL_SIZE:
	case RT5033FG_VGCOMP_IDLE_SIZE:
		rc = get_parameters(lbuf, param, 2);
		if (rc < 0)
			return rc;
		new_vgcomp_soc_offset_data(comp_offset_type, fuelgauge,
					   param[0], param[1]);
		prv_data_header[prv_data->id + 1].counter = 0;
		break;
	case RT5033FG_OFFSET_CHARGING_DATA:
	case RT5033FG_OFFSET_DISCHARGING_DATA:
	case RT5033FG_OFFSET_SPECIAL_DATA:
	case RT5033FG_OFFSET_LOW_POWER_DATA:
		index = comp_offset_type - SOC_OFFSET_CHARGING;
		rc = get_parameters(lbuf, param, 3);
		if (rc < 0)
			return rc;
		data = fuelgauge->info.soc_offset[index].soc_offset_data +
		       prv_data->counter;
		data->voltage = param[0];
		data->temperature = param[1];
		data->offset = param[2];

		prv_data->counter++;
		break;
	case RT5033FG_VGCOMP_NORMAL_DATA:
	case RT5033FG_VGCOMP_IDLE_DATA:
		index = comp_offset_type - NORMAL_VG_COMP;
		rc = get_parameters(lbuf, param, 6);
		if (rc < 0)
			return rc;
		data = fuelgauge->info.vg_comp[index].vg_comp_data +
		       prv_data->counter;
		data->voltage = param[0];
		data->temperature = param[1];
		data->vg_comp.data[0] = param[2];
		data->vg_comp.data[1] = param[3];
		data->vg_comp.data[2] = param[4];
		data->vg_comp.data[3] = param[5];
		prv_data->counter++;
		break;
	case RT5033FG_PARAM_LOCK:
		rc = get_parameters(lbuf, param, 1);
		if (rc < 0)
			return rc;
		if (param[0]) {
			if (prv_data->counter == 0)
				mutex_lock(&fuelgauge->info.param_lock);
			prv_data->counter = 1;
		} else {
			if (prv_data->counter == 1)
				mutex_unlock(&fuelgauge->info.param_lock);
			prv_data->counter = 0;
		}
		break;
	case RT5033FG_VGCOMP_IP_ORDER:
		rc = get_parameters(lbuf, param, 2);
		if (rc < 0)
			return rc;
		fuelgauge->info.vg_comp_interpolation_order[0] = param[0];
		fuelgauge->info.vg_comp_interpolation_order[1] = param[1];
		break;
	case RT5033FG_OFFSET_IP_ORDER:
		rc = get_parameters(lbuf, param, 2);
		if (rc < 0)
			return rc;
		fuelgauge->info.offset_interpolation_order[0] = param[0];
		fuelgauge->info.offset_interpolation_order[1] = param[1];
		break;
	case RT5033FG_CRATE_IDLE_THRES:
		rc = get_parameters(lbuf, param, 1);
		if (rc < 0)
			return rc;
		fuelgauge->info.crate_idle_thres = param[0];
		break;
	default:
		BUG();
	}

	return cnt;
}

static ssize_t rt5033_fg_debug_read(struct file *filp, char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	rt5033_dbg_private_data_t *prv_data = filp->private_data;
	rt5033_dbg_private_data_t *prv_data_header;
	struct sec_fuelgauge_info *fuelgauge = prv_data->fuelgague;
	data_point_t *data;
	char *lbuf = fuelgauge->info.dbg_out_buffer;
	int index;
	int comp_offset_type;
	int i = 0, j = 0;
	int data_size;
	lbuf[0] = '\0';
	comp_offset_type = dentry_id_to_comp_offset_type[prv_data->id];
	prv_data_header = prv_data - prv_data->id;
	switch(prv_data->id) {
	case RT5033FG_OFFSET_CHARGING_SIZE:
	case RT5033FG_OFFSET_DISCHARGING_SIZE:
	case RT5033FG_OFFSET_SPECIAL_SIZE:
	case RT5033FG_OFFSET_LOW_POWER_SIZE:
		index = comp_offset_type - SOC_OFFSET_CHARGING;
		snprintf(lbuf, RT5033_DBG_OUT_BUF_SIZE, "%d %d\n",
			 fuelgauge->info.soc_offset[index].soc_voltNR,
			 fuelgauge->info.soc_offset[index].tempNR);

		break;
	case RT5033FG_VGCOMP_NORMAL_SIZE:
	case RT5033FG_VGCOMP_IDLE_SIZE:
		index = comp_offset_type - NORMAL_VG_COMP;
		snprintf(lbuf, RT5033_DBG_OUT_BUF_SIZE, "%d %d\n",
			 fuelgauge->info.vg_comp[index].voltNR,
			 fuelgauge->info.vg_comp[index].tempNR);
		break;
	case RT5033FG_OFFSET_CHARGING_DATA:
	case RT5033FG_OFFSET_DISCHARGING_DATA:
	case RT5033FG_OFFSET_SPECIAL_DATA:
	case RT5033FG_OFFSET_LOW_POWER_DATA:
		index = comp_offset_type - SOC_OFFSET_CHARGING;
		data_size = fuelgauge->info.soc_offset[index].soc_voltNR *
			    fuelgauge->info.soc_offset[index].tempNR;
		if (data_size == 0)
			snprintf(lbuf, RT5033_DBG_OUT_BUF_SIZE, "no data\n");

		data = fuelgauge->info.soc_offset[index].soc_offset_data;
		for (i = 0; i < data_size; i++, data++) {
			j += snprintf(lbuf + j, RT5033_DBG_OUT_BUF_SIZE - j,
				      "%d %d %d\n", data->voltage,
				      data->temperature, data->offset);
		}

		break;
	case RT5033FG_VGCOMP_NORMAL_DATA:
	case RT5033FG_VGCOMP_IDLE_DATA:
		index = comp_offset_type - NORMAL_VG_COMP;
		data_size = fuelgauge->info.vg_comp[index].voltNR *
			    fuelgauge->info.vg_comp[index].tempNR;
		data = fuelgauge->info.vg_comp[index].vg_comp_data;
		if (data_size == 0)
			snprintf(lbuf, RT5033_DBG_OUT_BUF_SIZE, "no data\n");
		for (i = 0; i < data_size; i++, data++) {
			j += snprintf(lbuf + j, RT5033_DBG_OUT_BUF_SIZE - j,
				      "%d %d 0x%x 0x%x 0x%x 0x%x\n",
				      data->voltage, data->temperature,
				      data->vg_comp.data[0],
				      data->vg_comp.data[1],
				      data->vg_comp.data[2],
				      data->vg_comp.data[3]);
		}

		break;
	case RT5033FG_PARAM_LOCK:
		snprintf(lbuf, RT5033_DBG_OUT_BUF_SIZE, "%d\n",
			 prv_data->counter);
		break;
	case RT5033FG_VGCOMP_IP_ORDER:
		snprintf(lbuf, RT5033_DBG_OUT_BUF_SIZE, "%d %d\n",
			 fuelgauge->info.vg_comp_interpolation_order[0],
			 fuelgauge->info.vg_comp_interpolation_order[1]);
		break;
	case RT5033FG_OFFSET_IP_ORDER:
		snprintf(lbuf, RT5033_DBG_OUT_BUF_SIZE, "%d %d\n",
			 fuelgauge->info.offset_interpolation_order[0],
			 fuelgauge->info.offset_interpolation_order[1]);
		break;
	case RT5033FG_CRATE_IDLE_THRES:
		snprintf(lbuf, RT5033_DBG_OUT_BUF_SIZE, "%d\n",
			 fuelgauge->info.crate_idle_thres);
		break;
	default:
		BUG();
	}
	return simple_read_from_buffer(ubuf, count, ppos, lbuf, strlen(lbuf));
}

static const struct file_operations rt5033_fg_debug_ops = {
	.open = rt5033_fg_debug_open,
	.write = rt5033_fg_debug_write,
	.read = rt5033_fg_debug_read
};

static const char *dbgfs_names[] = {
	[RT5033FG_OFFSET_CHARGING_SIZE] = "offset_chg_size",
	[RT5033FG_OFFSET_CHARGING_DATA] = "offset_chg_data",
	[RT5033FG_OFFSET_DISCHARGING_SIZE] = "offset_dischg_size",
	[RT5033FG_OFFSET_DISCHARGING_DATA] = "offset_dischg_data",
	[RT5033FG_OFFSET_SPECIAL_SIZE] = "offset_spcl_size",
	[RT5033FG_OFFSET_SPECIAL_DATA] = "offset_spcl_data",
	[RT5033FG_OFFSET_LOW_POWER_SIZE] = "offset_low_power_size",
	[RT5033FG_OFFSET_LOW_POWER_DATA] = "offset_low_power_data",
	[RT5033FG_VGCOMP_NORMAL_SIZE] = "vgcomp_normal_size",
	[RT5033FG_VGCOMP_NORMAL_DATA] = "vgcomp_normal_data",
	[RT5033FG_VGCOMP_IDLE_SIZE] = "vgcomp_idle_size",
	[RT5033FG_VGCOMP_IDLE_DATA] = "vgcomp_idle_data",
	[RT5033FG_PARAM_LOCK] = "param_lock",
	[RT5033FG_VGCOMP_IP_ORDER] = "vgcomp_ip_order",
	[RT5033FG_OFFSET_IP_ORDER] = "offset_ip_order",
	[RT5033FG_CRATE_IDLE_THRES] = "crate_idle_thres",
};

#define DECL_RT5033FG_PRV_DATA(_id) [_id] = {.id = _id, .counter = 0,}

static rt5033_dbg_private_data_t rt5033_dbg_private_data[] = {
	DECL_RT5033FG_PRV_DATA(RT5033FG_OFFSET_CHARGING_SIZE),
	DECL_RT5033FG_PRV_DATA(RT5033FG_OFFSET_CHARGING_DATA),
	DECL_RT5033FG_PRV_DATA(RT5033FG_OFFSET_DISCHARGING_SIZE),
	DECL_RT5033FG_PRV_DATA(RT5033FG_OFFSET_DISCHARGING_DATA),
	DECL_RT5033FG_PRV_DATA(RT5033FG_OFFSET_SPECIAL_SIZE),
	DECL_RT5033FG_PRV_DATA(RT5033FG_OFFSET_SPECIAL_DATA),
	DECL_RT5033FG_PRV_DATA(RT5033FG_OFFSET_LOW_POWER_SIZE),
	DECL_RT5033FG_PRV_DATA(RT5033FG_OFFSET_LOW_POWER_DATA),
	DECL_RT5033FG_PRV_DATA(RT5033FG_VGCOMP_NORMAL_SIZE),
	DECL_RT5033FG_PRV_DATA(RT5033FG_VGCOMP_NORMAL_DATA),
	DECL_RT5033FG_PRV_DATA(RT5033FG_VGCOMP_IDLE_SIZE),
	DECL_RT5033FG_PRV_DATA(RT5033FG_VGCOMP_IDLE_DATA),
	DECL_RT5033FG_PRV_DATA(RT5033FG_PARAM_LOCK),
	DECL_RT5033FG_PRV_DATA(RT5033FG_VGCOMP_IP_ORDER),
	DECL_RT5033FG_PRV_DATA(RT5033FG_OFFSET_IP_ORDER),
	DECL_RT5033FG_PRV_DATA(RT5033FG_CRATE_IDLE_THRES),
};

static void rt5033_fg_create_debug_files(struct sec_fuelgauge_info *fuelgauge)
{
	int i;
	fuelgauge->info.dir_dentry = debugfs_create_dir("rt5033fg_dbg", 0);
	if (IS_ERR(fuelgauge->info.dir_dentry)) {
		pr_err("%s : cannot create rt5033fg_dbg\n", __func__);
		return;
	}
	BUG_ON(ARRAY_SIZE(dbgfs_names) != ARRAY_SIZE(rt5033_dbg_private_data));
	BUG_ON(ARRAY_SIZE(dbgfs_names) != RT5033FG_DENTRY_NR);
	for (i = 0; i < ARRAY_SIZE(dbgfs_names); ++i) {
		rt5033_dbg_private_data[i].fuelgague = fuelgauge;
		fuelgauge->info.file_dentries[i] =
			debugfs_create_file(dbgfs_names[i],
					    S_IFREG | S_IRUGO, fuelgauge->info.dir_dentry,
					    &rt5033_dbg_private_data[i], &rt5033_fg_debug_ops);
	}
}
#endif /* #ifdef CONFIG_DEBUG_FS */

bool sec_hal_fg_init(struct i2c_client *client)
{
    int size_x,size_y;
    int i, index;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	struct battery_data_t *battery_data;
	pr_info("rt5033 hal fg init...\n");
	mutex_init(&fuelgauge->info.param_lock);
	mutex_lock(&fuelgauge->info.param_lock);
	if (client->dev.of_node) {
	    // Load battery data from DTS
	    rt5033_fg_parse_dt(fuelgauge);

	} else {
		// Copy battery data from platform data
		battery_data = &get_battery_data(fuelgauge);
		fuelgauge->info.crate_idle_thres =
				battery_data->crate_idle_thres;
		fuelgauge->info.battery_type =
			battery_data->battery_type;
		fuelgauge->info.vg_comp_interpolation_order[0] =
			battery_data->vg_comp_interpolation_order[0];
		fuelgauge->info.vg_comp_interpolation_order[1] =
			battery_data->vg_comp_interpolation_order[1];
		fuelgauge->info.offset_interpolation_order[0] =
			battery_data->offset_interpolation_order[0];
		fuelgauge->info.offset_interpolation_order[1] =
			battery_data->offset_interpolation_order[1];
		for (i = NORMAL_VG_COMP; i <= IDLE_VG_COMP; ++i) {
			index = i - NORMAL_VG_COMP;
			size_x = battery_data->vg_comp[index].voltNR;
			size_y = battery_data->vg_comp[index].tempNR;
			dev_dbg(&client->dev,
				"%s line %d: index = %d, size_x = %d, size_y = %d\n",
				__func__, __LINE__, index, size_x, size_y);
			new_vgcomp_soc_offset_data(i, fuelgauge, size_x, size_y);
			if (fuelgauge->info.vg_comp[index].vg_comp_data) {
				memcpy(fuelgauge->info.vg_comp[index].vg_comp_data,
				       battery_data->vg_comp[index].vg_comp_data,
				       sizeof(data_point_t) * size_x * size_y);
			}
		}
		for (i = SOC_OFFSET_CHARGING; i <= SOC_OFFSET_LOW_POWER; ++i) {
			index = i - SOC_OFFSET_CHARGING;
			size_x = battery_data->soc_offset[index].soc_voltNR;
			size_y = battery_data->soc_offset[index].tempNR;
			dev_dbg(&client->dev,
				"%s line %d: index = %d, size_x = %d, size_y = %d\n",
				__func__, __LINE__, index, size_x, size_y);
			new_vgcomp_soc_offset_data(i, fuelgauge, size_x, size_y);
			if (fuelgauge->info.soc_offset[index].soc_offset_data) {
				memcpy(fuelgauge->info.soc_offset[index].soc_offset_data,
				       battery_data->soc_offset[index].soc_offset_data,
				       sizeof(data_point_t) * size_x * size_y);
			}
		}
	}
	//struct battery_data_t *battery_data = &get_battery_data(fuelgauge);

	rt5033_fg_init(client);
	rt5033_pr_ver_info(client);
	fuelgauge->info.init_once = true;
	fuelgauge->info.pre_soc = 50;
	fuelgauge->info.move_avg_offset_cnt = 0;
	fuelgauge->info.volt_alert_flag = false;
	fuelgauge->info.soc_alert_flag = false;
	fuelgauge->info.bat_pres_flag = true;
	fuelgauge->info.offs_speci_case = false;
	fuelgauge->info.temperature = 250;
	fuelgauge->info.entry_suspend_flag = false;

#ifdef CONFIG_DEBUG_FS
    rt5033_fg_create_debug_files(fuelgauge);
#endif
    mutex_unlock(&fuelgauge->info.param_lock);
    pr_info("rt5033 hal fg init OK\n");
	return true;
}

bool sec_hal_fg_suspend(struct i2c_client *client)
{

	int volt_suspend;
	int temp_suspend;
	int ret;
	vg_comp_data_t vgcomp;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	fuelgauge->info.entry_suspend_flag = true;
	/* vgcomp */
	volt_suspend = fg_get_avg_volt(client);
	temp_suspend = fuelgauge->info.temperature/10;

	// report FG_AVG_VOLT & FG_TEMP for debugging.
	pr_info("%s : FG_AVG_VOLT = %d, FG_TEMP = %d\n", __func__,
            volt_suspend, fuelgauge->info.temperature);

	vgcomp = rt5033_fg_get_vgcomp(fuelgauge, VGCOMP_IDLE,
                                  volt_suspend, fuelgauge->info.temperature);
	ret = (vgcomp.data[0]<<8) | vgcomp.data[1];
	rt5033_fg_i2c_write_word(client, RT5033_VGCOMP1, ret);
	ret = (vgcomp.data[2]<<8) | vgcomp.data[3];
	rt5033_fg_i2c_write_word(client, RT5033_VGCOMP3, ret);

	// report VGCOMP1 ~ 4 for debugging.
	ret = rt5033_fg_i2c_read_word(client, RT5033_VGCOMP1);
	pr_info("%s : FG_VGCOMP1_2_Idle = 0x%04x\n", __func__, ret);
	ret = rt5033_fg_i2c_read_word(client, RT5033_VGCOMP3);
	pr_info("%s : FG_VGCOMP3_4_Idle = 0x%04x\n", __func__, ret);

	return true;
}

bool sec_hal_fg_resume(struct i2c_client *client)
{
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
#if 0
	int ret;
	/* disable hibernate mode */
	ret = rt5033_fg_i2c_read_word(client, RT5033_CONFIG_MSB);
	if(ret & 0x8000)
		rt5033_fg_i2c_write_word(client,
			RT5033_MFA_MSB, MFA_CMD_EXIT_HIB);
#endif
	fuelgauge->info.init_once = true;
	fuelgauge->info.entry_suspend_flag = false;
	return true;
}

bool sec_hal_fg_fuelalert_init(struct i2c_client *client, int soc)
{
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	int ret;

	ret = rt5033_fg_i2c_read_word(client, RT5033_FG_IRQ_CTRL);
	/* enable volt, soc and battery presence alert irq; clear volt and soc alert status via i2c */
	ret |= 0x1f00;
	ret &= 0xfffc;
	rt5033_fg_i2c_write_word(client,RT5033_FG_IRQ_CTRL,ret);
    fuelgauge->info.irq_ctrl = ret;
	/* set volt and soc alert threshold */
	ret = 0;
	ret = VOLT_ALRT_TH << 8;
	ret += soc;
	rt5033_fg_i2c_write_word(client, RT5033_VOLT_ALRT_TH, ret);

	/* reset soc alert flag */
	fuelgauge->info.soc_alert_flag = false;

	return true;
}

bool sec_hal_fg_is_fuelalerted(struct i2c_client *client)
{
	int ret;
	bool retval;
	union power_supply_propval val;
	struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);

	pr_info("%s ------------------------------------------\n ",__func__);

	ret = rt5033_fg_i2c_read_word(client, RT5033_FG_IRQ_CTRL);
    if (ret < 0) {
        pr_info("rt5033 fg is alerted : i2c read error(%d)\n", ret);
        return false;
    }
    retval = (ret & 0x03) ? true : false;
    fuelgauge->info.irq_ctrl = ret & (~0x04);
    if (ret & 0x04) { /* battery detection INT */
        ret = rt5033_fg_i2c_read_word(client, RT5033_FG_IRQ_CTRL);
        ret &= (~0x04);
        rt5033_fg_i2c_write_word(client, RT5033_FG_IRQ_CTRL, ret);
        ret = rt5033_fg_i2c_read_word(client, RT5033_CONFIG_MSB);
		if(ret&0x0002)
			fuelgauge->info.bat_pres_flag = true;   // battery inserted
		else
			fuelgauge->info.bat_pres_flag = false;	// battery removed

		/* send Quick Sensing command to 5033FG */
		if (fuelgauge->info.bat_pres_flag) {
			dev_info(&client->dev, "%s: Battery Inserted! Do QS...\n",
				 __func__);
			msleep(1000);
			rt5033_fg_i2c_write_word(client, 0x06, 0x8000);
			if (!sec_bat_check_jig_status()) {
				val.intval = true;
				psy_do_property("battery", set,
					POWER_SUPPLY_PROP_PRESENT, val);
			}
		} else {
			pr_info("%s: Battery Removed !!!\n",__func__);
			if (!sec_bat_check_jig_status()) {
				val.intval = false;
				psy_do_property("battery", set,
					POWER_SUPPLY_PROP_PRESENT, val);
			}
		}
	}

	ret &= (~0x04);
	rt5033_fg_i2c_write_word(client, RT5033_FG_IRQ_CTRL, ret);

	return retval;
}

bool sec_hal_fg_fuelalert_process(void *irq_data, bool is_fuel_alerted)
{

	struct sec_fuelgauge_info *fuelgauge = irq_data;
    struct i2c_client *client = fuelgauge->client;
	int ret;
	int ret2;
	int temp;

	ret = fuelgauge->info.irq_ctrl;

	/* soc alert process */
	if (ret & 0x01)	{
		temp = ret;
		// disable soc alert
		ret2 = rt5033_fg_i2c_read_word(client, RT5033_VOLT_ALRT_TH);
		ret2 &= 0xff00;
		rt5033_fg_i2c_write_word(client, RT5033_VOLT_ALRT_TH, ret2);
		// clear soc alert status
	  temp &= 0xfffe;
	  rt5033_fg_i2c_write_word(client, RT5033_FG_IRQ_CTRL, temp);
	  // set soc alert flag
	  fuelgauge->info.soc_alert_flag = true;
	}

	/* voltage alert process */
	if (ret & 0x02)	{
		temp = ret;
		// disable voltage alert
		ret2 = rt5033_fg_i2c_read_word(client, RT5033_VOLT_ALRT_TH);
		ret2 &= 0x00ff;
		rt5033_fg_i2c_write_word(client, RT5033_VOLT_ALRT_TH, ret2);
		// clear voltage alert status
	  temp &= 0xfffd;
	  rt5033_fg_i2c_write_word(client, RT5033_FG_IRQ_CTRL, temp);
	  rt5033_fg_i2c_write_word(client, RT5033_MFA_MSB, MFA_CMD_VALRT_SOC);
	 fuelgauge->info.volt_alert_flag = true;
	}
	return true;
}

bool sec_hal_fg_full_charged(struct i2c_client *client)
{
    struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
    fuelgauge->info.flag_full_charge = 1;
	return true;
}

bool sec_hal_fg_reset(struct i2c_client *client)
{
	rt5033_fg_i2c_write_word(client, 0x06, 0x4000);
	return true;
}

bool sec_hal_fg_get_property(struct i2c_client *client,
			     enum power_supply_property psp,
			     union power_supply_propval *val)
{
	union power_supply_propval value , value2;
	struct sec_fuelgauge_info *fuelgauge =
		i2c_get_clientdata(client);

	psy_do_property("rt5033-charger", get,
			POWER_SUPPLY_PROP_STATUS, value);
	fuelgauge->info.flag_full_charge =
		(value.intval == POWER_SUPPLY_STATUS_FULL) ? 1 : 0;
	fuelgauge->info.flag_chg_status =
		(value.intval == POWER_SUPPLY_STATUS_CHARGING) ? 1 : 0;

	switch (psp) {
	/* Cell voltage (VCELL, mV) */
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = fg_get_vbat(client);
		break;
	/* Additional Voltage Information (mV) */
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		switch (val->intval) {
		case SEC_BATTEY_VOLTAGE_AVERAGE:
			val->intval = fg_get_avg_volt(client);
			break;
		case SEC_BATTEY_VOLTAGE_OCV:
			val->intval = fg_get_ocv(client);
			break;
		}
		break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = rt5033_fg_get_batt_present(client);
			pr_info("%s batt id = %d \n",__func__,val->intval);
			break;
			/* Current (mA) */
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			/* Average Current (mA) */
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			psy_do_property("battery", get,
			POWER_SUPPLY_PROP_HEALTH, value);
			psy_do_property("battery", get,
			POWER_SUPPLY_PROP_ONLINE, value2);

			if ((value.intval != POWER_SUPPLY_HEALTH_GOOD) || (value2.intval == POWER_SUPPLY_TYPE_USB))
				val->intval = -1;
			else
				val->intval = 0;
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			val->intval =
				(fuelgauge->info.batt_soc >= 1000) ? true : false;
			break;
			/* SOC (%) */
		case POWER_SUPPLY_PROP_CAPACITY:
			/* RT5033 F/G unit is 0.1%, raw ==> convert the unit to 0.01% */
			if (val->intval == SEC_FUELGAUGE_CAPACITY_TYPE_RAW)
				val->intval = fg_get_soc(client) * 10;
			else
				val->intval = fg_get_soc(client);
			break;
			/* Battery Temperature */
		case POWER_SUPPLY_PROP_TEMP:
			/* Target Temperature */
		case POWER_SUPPLY_PROP_TEMP_AMBIENT:
			val->intval = fuelgauge->info.temperature;
			break;
		default:
			return false;
	}
	return true;
}

bool sec_hal_fg_set_property(struct i2c_client *client,
			     enum power_supply_property psp,
			     const union power_supply_propval *val)
{
    struct sec_fuelgauge_info *fuelgauge = i2c_get_clientdata(client);
	switch (psp) {
		/* Battery Temperature */
	case POWER_SUPPLY_PROP_TEMP:
                fuelgauge->info.temperature = val->intval;
                break;
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
                break;
	default:
		return false;
	}
	return true;
}

ssize_t sec_hal_fg_show_attrs(struct device *dev,
				const ptrdiff_t offset, char *buf)
{

	struct power_supply *psy = dev_get_drvdata(dev);
	struct sec_fuelgauge_info *fg =
		container_of(psy, struct sec_fuelgauge_info, psy_fg);
    struct sec_fg_info *info = &fg->info;
	int i = 0;

	switch (offset) {
/*	case FG_REG: */
/*		break; */
	case FG_DATA:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%02x%02x\n",
			info->reg_data[1], info->reg_data[0]);
		break;
	default:
		i = -EINVAL;
		break;
	}
	return i;
}

ssize_t sec_hal_fg_store_attrs(struct device *dev,
				const ptrdiff_t offset,
				const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct sec_fuelgauge_info *fg =
		container_of(psy, struct sec_fuelgauge_info, psy_fg);
    struct sec_fg_info *info = &fg->info;
	int ret = 0;
	int x = 0;
	int data;

	switch (offset) {
	case FG_REG:
		if (sscanf(buf, "%x\n", &x) == 1) {
			info->reg_addr = x;
			data = rt5033_fg_i2c_read_word(fg->client,
				info->reg_addr);
			info->reg_data[0] = data&0xff;
			info->reg_data[1] = (data>>8)&0xff;
			dev_dbg(info->dev,
				"%s: (read) addr = 0x%x, data = 0x%02x%02x\n",
				 __func__, info->reg_addr,
				 info->reg_data[1], info->reg_data[0]);
			ret = count;
		}
		break;
	case FG_DATA:
		if (sscanf(buf, "%x\n", &x) == 1) {
			info->reg_data[0] = (x & 0xFF00) >> 8;
			info->reg_data[1] = (x & 0x00FF);
			dev_dbg(info->dev,
				"%s: (write) addr = 0x%x, data = 0x%02x%02x\n",
				__func__, info->reg_addr, info->reg_data[1], info->reg_data[0]);

			rt5033_fg_i2c_write_word(fg->client,
				info->reg_addr, x);
			ret = count;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
