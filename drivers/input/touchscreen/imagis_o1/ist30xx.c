/*
 *  Copyright (C) 2010,Imagis Technology Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include <linux/input/tsp_ta_callback.h>
#include "ist30xx.h"
#include "ist30xx_update.h"
#include "ist30xx_tracking.h"

#if IST30XX_DEBUG
#include "ist30xx_misc.h"
#endif
#if IST30XX_CMCS_TEST
#include "ist30xx_cmcs.h"
#endif
#if SEC_FACTORY_MODE
#include "ist30xx_sec.h"
extern int sec_touch_sysfs(struct ist30xx_data *data);
extern int sec_fac_cmd_init(struct ist30xx_data *data);
#endif

#define MAX_ERR_CNT             (100)

#if IST30XX_USE_KEY
int ist30xx_key_code[] = { 0, KEY_RECENT, KEY_BACK };
#endif

DEFINE_MUTEX(ist30xx_mutex);

volatile bool ist30xx_irq_working = false;
static bool ist30xx_initialized = 0;

struct ist30xx_data *ts_data;
EXPORT_SYMBOL(ts_data);

static struct delayed_work work_reset_check;
#if IST30XX_EVENT_MODE
static struct delayed_work work_noise_protect, work_debug_algorithm;
#endif

static int ist30xx_power_status = -1;
static int ist30xx_ta_status = -1;
static int ist30xx_noise_mode = -1;

#if IST30XX_INTERNAL_BIN && IST30XX_UPDATE_BY_WORKQUEUE
static struct delayed_work work_fw_update;
#endif

int ist30xx_report_rate = -1;
int ist30xx_idle_rate = -1;

u32 event_ms = 0, timer_ms = 0;
u32 ist30xx_scan_count = 0;
int ist30xx_scan_retry = 0;

#if IST30XX_EVENT_MODE
static struct timer_list event_timer;
static struct timespec t_current;               // ns
int timer_period_ms = 500;                      // 0.5sec
#define EVENT_TIMER_INTERVAL	(HZ * timer_period_ms / 1000)
#endif  // IST30XX_EVENT_MODE

#if IST30XX_DEBUG
extern TSP_INFO ist30xx_tsp_info;
extern TKEY_INFO ist30xx_tkey_info;
#endif

int ist30xx_dbg_level = IST30XX_DEBUG_LEVEL;
void tsp_printk(int level, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;
	int r;

	if (ist30xx_dbg_level < level)
		return;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	r = printk("%s %pV", IST30XX_DEBUG_TAG, &vaf);

	va_end(args);
}

long get_milli_second(void)
{
#if IST30XX_EVENT_MODE
	ktime_get_ts(&t_current);

	return t_current.tv_sec * 1000 + t_current.tv_nsec / 1000000;
#else
	return 0;
#endif  // IST30XX_EVENT_MODE
}

int ist30xx_intr_wait(long ms)
{
	long start_ms = get_milli_second();
	long curr_ms = 0;

	while (1) {
		if (!ist30xx_irq_working)
			break;

		curr_ms = get_milli_second();
		if ((curr_ms < 0) || (start_ms < 0) || (curr_ms - start_ms > ms)) {
			tsp_info("%s() timeout(%dms)\n", __func__, ms);
			return -EPERM;
		}

		msleep(2);
	}
	return 0;
}

void ist30xx_disable_irq(struct ist30xx_data *data)
{
	if (likely(data->irq_enabled)) {
		ist30xx_tracking(TRACK_INTR_DISABLE);
		disable_irq(data->client->irq);
		data->irq_enabled = 0;
		data->status.event_mode = false;
	}
}

void ist30xx_enable_irq(struct ist30xx_data *data)
{
	if (likely(!data->irq_enabled)) {
		ist30xx_tracking(TRACK_INTR_ENABLE);
		enable_irq(data->client->irq);
		msleep(10);
		data->irq_enabled = 1;
		data->status.event_mode = true;
	}
}

int ist30xx_max_error_cnt = MAX_ERR_CNT;
int ist30xx_error_cnt = 0;
void ist30xx_scheduled_reset(void)
{
	if (likely(ist30xx_initialized))
		schedule_delayed_work(&work_reset_check, 0);
}

static void ist30xx_request_reset(void)
{
	ist30xx_error_cnt++;
	if (unlikely(ist30xx_error_cnt >= ist30xx_max_error_cnt)) {
		tsp_info("%s()\n", __func__);
		ist30xx_scheduled_reset();
		ist30xx_error_cnt = 0;
	}
}

#define NOISE_MODE_TA       (0)
#define NOISE_MODE_CALL     (1)
#define NOISE_MODE_COVER    (2)
#define NOISE_MODE_POWER    (8)
void ist30xx_start(struct ist30xx_data *data)
{
	if (ist30xx_initialized) {
#if IST30XX_EVENT_MODE
        ist30xx_scan_count = 0;
        ist30xx_scan_retry = 0;
		mod_timer(&event_timer, get_jiffies_64() + EVENT_TIMER_INTERVAL * 2);
#endif
	}

	ist30xx_tracking(ist30xx_ta_status ?
			 TRACK_CMD_TACON : TRACK_CMD_TADISCON);

	ist30xx_noise_mode &= ~(1 << NOISE_MODE_TA);
	ist30xx_noise_mode |= (ist30xx_ta_status << NOISE_MODE_TA);

	ist30xx_noise_mode &= ~(1 << NOISE_MODE_POWER);
	ist30xx_noise_mode |= (ist30xx_power_status << NOISE_MODE_POWER);

	ist30xx_noise_mode |= (TSP_LOCAL_CODE << 16);
	ist30xx_write_cmd(data->client, CMD_SET_NOISE_MODE, ist30xx_noise_mode);

	tsp_info("%s(), local : %d, mode : 0x%x\n", __func__,
		 (ist30xx_noise_mode >> 16) & 0xFFFF, ist30xx_noise_mode & 0xFFFF);

	if (ist30xx_report_rate >= 0) {
		ist30xx_write_cmd(data->client, CMD_SET_REPORT_RATE,
				  ist30xx_report_rate);
		tsp_info(" reporting rate : %dus\n", ist30xx_report_rate);
	}

	if (ist30xx_idle_rate >= 0) {
		ist30xx_write_cmd(data->client, CMD_SET_IDLE_TIME, ist30xx_idle_rate);
		tsp_info(" idle rate : %dus\n", ist30xx_idle_rate);
	}

	ist30xx_cmd_start_scan(data->client);
}


int ist30xx_get_ver_info(struct ist30xx_data *data)
{
	int ret;

	data->fw.prev_core_ver = data->fw.core_ver;
	data->fw.prev_param_ver = data->fw.param_ver;
	data->fw.core_ver = data->fw.param_ver = 0;

	ret = ist30xx_read_cmd(data->client, CMD_GET_FW_VER, &data->fw.core_ver);
	if (unlikely(ret))
		return ret;

	ret = ist30xx_read_cmd(data->client, CMD_GET_PARAM_VER,
			       &data->fw.param_ver);
	if (unlikely(ret))
		return ret;

	ret = ist30xx_read_cmd(data->client, CMD_GET_SUB_VER, &data->fw.sub_ver);
	if (unlikely(ret))
		return ret;

	tsp_info("IC version read core: %x, param: %x, sub: %x\n",
		 data->fw.core_ver, data->fw.param_ver, data->fw.sub_ver);

	return 0;
}

#if IST30XX_DEBUG
void ist30xx_print_info(void)
{
	TSP_INFO *tsp = &ist30xx_tsp_info;
	TKEY_INFO *tkey = &ist30xx_tkey_info;

	tsp_info("*** TSP/TKEY info ***\n");
	tsp_info("TSP info: \n");
	tsp_info(" finger num: %d\n", tsp->finger_num);
	tsp_info(" dir swap: %d, flip x: %d, y: %d\n",
		 tsp->dir.swap_xy, tsp->dir.flip_x, tsp->dir.flip_y);
	tsp_info(" baseline: %d\n", tsp->baseline);
	tsp_info(" ch_num tx: %d, rx: %d\n", tsp->ch_num.tx, tsp->ch_num.rx);
	tsp_info(" screen tx: %d, rx: %d\n", tsp->screen.tx, tsp->screen.rx);
	tsp_info(" width: %d, height: %d\n", tsp->width, tsp->height);
	tsp_info(" gtx num: %d, ch [1]: %d, [2]: %d, [3]: %d, [4]: %d\n",
		 tsp->gtx.num, tsp->gtx.ch_num[0], tsp->gtx.ch_num[1],
		 tsp->gtx.ch_num[2], tsp->gtx.ch_num[3]);
	tsp_info(" node len: %d\n", tsp->node.len);
	tsp_info("TKEY info: \n");
	tsp_info(" enable: %d, key num: %d\n", tkey->enable, tkey->key_num);
	tsp_info(" ch [0]: %d,%d [1]: %d,%d [2]: %d,%d [3]: %d,%d [4]: %d,%d\n",
		 tkey->ch_num[0].tx, tkey->ch_num[0].rx, tkey->ch_num[1].tx,
		 tkey->ch_num[1].rx, tkey->ch_num[2].tx, tkey->ch_num[2].rx,
		 tkey->ch_num[3].tx, tkey->ch_num[3].rx, tkey->ch_num[4].tx,
		 tkey->ch_num[4].rx);
	tsp_info(" baseline : %d\n", tkey->baseline);
}
#endif

#define CALIB_MSG_MASK          (0xF0000FFF)
#define CALIB_MSG_VALID         (0x80000CAB)
#define TRACKING_INTR_VALID     (0x127EA597)
u32 tracking_intr_value = TRACKING_INTR_VALID;
int ist30xx_get_info(struct ist30xx_data *data)
{
	int ret;
	u32 calib_msg;
	u32 ms;

	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);

#if IST30XX_INTERNAL_BIN
#if IST30XX_DEBUG
#if IST30XX_UPDATE_BY_WORKQUEUE
	ist30xx_get_update_info(data, data->fw.buf, data->fw.buf_size);
#endif
	ist30xx_get_tsp_info(data);
#endif  // IST30XX_DEBUG
#else

	ret = ist30xx_get_ver_info(data);
	if (unlikely(ret))
		goto get_info_end;

#if IST30XX_DEBUG
	ret = ist30xx_tsp_update_info();
	if (unlikely(ret))
		goto get_info_end;

	ret = ist30xx_tkey_update_info();
	if (unlikely(ret))
		goto get_info_end;
#endif  // IST30XX_DEBUG
#endif  // IST30XX_INTERNAL_BIN

#if IST30XX_DEBUG
	ist30xx_print_info();
	data->max_fingers = ist30xx_tsp_info.finger_num;
	data->max_keys = ist30xx_tkey_info.key_num;
#endif  // IST30XX_DEBUG

	ret = ist30xx_read_cmd(ts_data->client, CMD_GET_CALIB_RESULT, &calib_msg);
	if (likely(ret == 0)) {
		tsp_info("calib status: 0x%08x\n", calib_msg);
		ms = get_milli_second();
		ist30xx_put_track_ms(ms);
		ist30xx_put_track(&tracking_intr_value, 1);
		ist30xx_put_track(&calib_msg, 1);
		if ((calib_msg & CALIB_MSG_MASK) != CALIB_MSG_VALID ||
			CALIB_TO_STATUS(calib_msg) > 0) {
			ist30xx_calibrate(IST30XX_FW_UPDATE_RETRY);
			ist30xx_reset(false);
		}
	}

#if (IST30XX_EVENT_MODE && IST30XX_CHECK_CALIB)
	if (likely(!data->status.update)) {
		ret = ist30xx_cmd_check_calib(data->client);
		if (likely(!ret)) {
			data->status.calib = 1;
			data->status.calib_msg = 0;
			event_ms = (u32)get_milli_second();
			data->status.event_mode = true;
		}
	}
#else
	ist30xx_start(ts_data);
#endif

#if !(IST30XX_INTERNAL_BIN)
get_info_end:
#endif
	if (likely(ret == 0))
		ist30xx_enable_irq(data);
	mutex_unlock(&ist30xx_mutex);

	return ret;
}

#if IST30XX_ENABLE_GESTURE
/* For Gesture */
#define GESTURE_MAGIC_STRING		(0x4170CF00)
#define GESTURE_MAGIC_MASK		(0xFFFFFF00)
#define GESTURE_MESSAGE_MASK		(~GESTURE_MAGIC_MASK)
#define PARSE_GESTURE_MESSAGE(n)  \
	((n & GESTURE_MAGIC_MASK) == GESTURE_MAGIC_STRING ? \
	(n & GESTURE_MESSAGE_MASK) : -EINVAL)

void ist30xx_gesture_cmd(struct ist30xx_data *data, int cmd)
{
	tsp_info("Gesture cmd: %d\n", cmd);

	switch (cmd) {
	case 0x01:
		input_report_key(data->input_dev, KEY_VOLUMEDOWN, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_VOLUMEDOWN, 0);
		input_sync(data->input_dev);
		break;;

	case 0x02:
		input_report_key(data->input_dev, KEY_VOLUMEUP, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_VOLUMEUP, 0);
		input_sync(data->input_dev);
		break;

	case 0x03:
		input_report_key(data->input_dev, KEY_PREVIOUSSONG, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_PREVIOUSSONG, 0);
		input_sync(data->input_dev);
		break;

	case 0x04:
		input_report_key(data->input_dev, KEY_NEXTSONG, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_NEXTSONG, 0);
		input_sync(data->input_dev);
		break;

	case 0x11:
		input_report_key(data->input_dev, KEY_PLAYPAUSE, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_PLAYPAUSE, 0);
		input_sync(data->input_dev);
		break;

	case 0x12:
		input_report_key(data->input_dev, KEY_SCREENLOCK, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_SCREENLOCK, 0);
		input_sync(data->input_dev);
		break;

	case 0x13:
		input_report_key(data->input_dev, KEY_PLAYPAUSE, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_PLAYPAUSE, 0);
		input_sync(data->input_dev);
		break;

	case 0x14:
		input_report_key(data->input_dev, KEY_PLAYPAUSE, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_PLAYPAUSE, 0);
		input_sync(data->input_dev);
		 break;

	case 0x21:
		input_report_key(data->input_dev, KEY_MUTE, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_MUTE, 0);
		input_sync(data->input_dev);
		break;

	case 0x31:
		ist30xx_reset(false);
		input_report_key(data->input_dev, KEY_POWER, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_POWER, 0);
		input_sync(data->input_dev);
		break;

	default:
		break;
	}
}
#endif

#define PRESS_MSG_MASK          (0x01)
#define MULTI_MSG_MASK          (0x02)
#define PRESS_MSG_KEY           (0x6)

#define TOUCH_DOWN_MESSAGE      ("P")
#define TOUCH_UP_MESSAGE        ("R")
#define TOUCH_MOVE_MESSAGE      (" ")

bool tsp_touched[IST30XX_MAX_MT_FINGERS] = { false, };
int tsp_count[IST30XX_MAX_MT_FINGERS] = { 0, };
bool tkey_pressed[IST30XX_MAX_KEYS] = { false, };

void print_tsp_event(finger_info *finger)
{
	int idx = finger->bit_field.id - 1;
	bool press;

	press = PRESSED_FINGER(ts_data->t_status, finger->bit_field.id);

	if (press) {
		if (tsp_touched[idx] == false) { // touch down
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
			tsp_info("%s[%d] (%d, %d)\n",
				 TOUCH_DOWN_MESSAGE, idx,
				 finger->bit_field.x, finger->bit_field.y);
#else
			tsp_info("%s[%d]\n",
				 TOUCH_DOWN_MESSAGE, idx);
#endif
			tsp_touched[idx] = true;
		} else {		    // touch move
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
			tsp_debug("%s[%d] (%d, %d)\n",
				 TOUCH_MOVE_MESSAGE, idx,
				 finger->bit_field.x, finger->bit_field.y);
#else
			tsp_debug("%s[%d]\n",
				 TOUCH_MOVE_MESSAGE, idx);
#endif
			tsp_count[idx]++;
		}
	} else {
		if (tsp_touched[idx] == true) { // touch up
			tsp_info("%s[%d] M(%d) V(%02x)\n",
				 TOUCH_UP_MESSAGE, idx, tsp_count[idx],
				 ts_data->fw.param_ver);
			tsp_touched[idx] = false;
			tsp_count[idx] = 0;
		}
	}
}

void print_tkey_event(int id)
{
	int idx = id - 1;
	bool press = PRESSED_KEY(ts_data->t_status, id);

	if (press) {
		if (tkey_pressed[idx] == false) { // tkey down
			tsp_notc("button %d %s\n", idx, TOUCH_DOWN_MESSAGE);
			tkey_pressed[idx] = true;
		}
	} else {
		if (tkey_pressed[idx] == true) { // tkey up
			tsp_notc("button %d %s\n", idx, TOUCH_UP_MESSAGE);
			tkey_pressed[idx] = false;
		}
	}
}

static void release_finger(int id)
{
	input_mt_slot(ts_data->input_dev, id - 1);
	input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, false);

	ist30xx_tracking(TRACK_POS_FINGER + id);
	tsp_info("%s() %d\n", __func__, id);

	tsp_touched[id - 1] = false;

	input_sync(ts_data->input_dev);
}

#define CANCEL_KEY  (0xFF)
#define RELEASE_KEY (0)
static void release_key(int id, u8 key_status)
{
	input_report_key(ts_data->input_dev, ist30xx_key_code[id], key_status);

	ist30xx_tracking(TRACK_POS_KEY + id);
	tsp_info("%s() key%d, status: %d\n", __func__, id, key_status);

	tkey_pressed[id - 1] = false;

	input_sync(ts_data->input_dev);
}

static void clear_input_data(struct ist30xx_data *data)
{
	int id = 1;
	u32 status;

	input_report_key(data->input_dev, BTN_TOUCH, 0);

	status = PARSE_FINGER_STATUS(data->t_status);
	while (status) {
		if (status & 1)
			release_finger(id);
		status >>= 1;
		id++;
	}

	id = 1;
	status = PARSE_KEY_STATUS(data->t_status);
	while (status) {
		if (status & 1)
			release_key(id, RELEASE_KEY);
		status >>= 1;
		id++;
	}
	data->t_status = 0;
}

static int check_report_fingers(struct ist30xx_data *data, int finger_counts)
{
	int i;
	finger_info *fingers = (finger_info *)data->fingers;

	/* current finger info */
	for (i = 0; i < finger_counts; i++) {
		if (unlikely((fingers[i].bit_field.x > IST30XX_MAX_X) ||
			     (fingers[i].bit_field.y > IST30XX_MAX_Y))) {
			tsp_warn("Invalid touch data - %d: %d(%d, %d), 0x%08x\n", i,
				 fingers[i].bit_field.id,
				 fingers[i].bit_field.x,
				 fingers[i].bit_field.y,
				 fingers[i].full_field);

			fingers[i].bit_field.id = 0;
			ist30xx_tracking(TRACK_POS_UNKNOWN);
			return -EPERM;
		}
	}

    return 0;
}

static int check_valid_coord(u32 *msg, int cnt)
{
	int ret = 0;
	u8 *buf = (u8 *)msg;
	u8 chksum1 = msg[0] >> 24;
	u8 chksum2 = 0;

	msg[0] &= 0x00FFFFFF;

	cnt *= IST30XX_DATA_LEN;

	while (cnt--)
		chksum2 += *buf++;

	if (chksum1 != chksum2) {
		tsp_err("intr chksum: %02x, %02x\n", chksum1, chksum2);
		ret = -EPERM;
	}

	return (chksum1 == chksum2) ? 0 : -EPERM;
}

static void report_input_data(struct ist30xx_data *data, int finger_counts,
			      int key_counts)
{
	int id;
	bool press = false;
	finger_info *fingers = (finger_info *)data->fingers;
	int idx = 0;
	u32 status;

	status = PARSE_FINGER_STATUS(data->t_status);
	for (id = 0; id < IST30XX_MAX_MT_FINGERS; id++) {
		press = (status & (1 << id)) ? true : false;

		input_mt_slot(data->input_dev, id);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, press);

		fingers[idx].bit_field.id = id + 1;
		print_tsp_event(&fingers[idx]);

		if (press == false)
			continue;

		input_report_abs(data->input_dev, ABS_MT_POSITION_X,
				 fingers[idx].bit_field.x);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
				 fingers[idx].bit_field.y);
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
				 fingers[idx].bit_field.area);
		input_report_key(data->input_dev, BTN_TOUCH, 1);
		idx++;
	}

#if IST30XX_USE_KEY
	status = PARSE_KEY_STATUS(data->t_status);
	for (id = 0; id < data->max_keys; id++) {
		press = (status & (1 << id)) ? true : false;

		input_report_key(data->input_dev, ist30xx_key_code[id + 1], press);

		print_tkey_event(id + 1);
	}
#endif  // IST30XX_USE_KEY

	if (finger_counts == 0)
		input_report_key(data->input_dev, BTN_TOUCH, 0);

	ist30xx_error_cnt = 0;
	ist30xx_scan_retry = 0;

	input_sync(data->input_dev);
}

/*
 * CMD : CMD_GET_COORD
 *
 *   1st  [31:24]   [23:21]   [20:16]   [15:12]   [11:10]   [9:0]
 *        Checksum  KeyCnt    KeyStatus FingerCnt Rsvd.     FingerStatus
 *   2nd  [31:28]   [27:24]   [23:12]   [11:0]
 *        ID        Area      X         Y
 */
u32 intr_debug_addr, intr_debug2_addr, intr_debug3_addr = 0;
u32 intr_debug_size, intr_debug2_size, intr_debug3_size = 0;
static irqreturn_t ist30xx_irq_thread(int irq, void *ptr)
{
	int i, ret;
	int key_cnt, finger_cnt, read_cnt;
	struct ist30xx_data *data = ts_data;
	int offset = 1;
	u32 msg[IST30XX_MAX_MT_FINGERS + offset];
	u32 ms;

	ist30xx_irq_working = true;

	if (unlikely(!data->irq_enabled))
		goto irq_end;

	ms = get_milli_second();

	memset(msg, 0, sizeof(msg));

	ret = ist30xx_get_position(data->client, msg, 1);
	if (unlikely(ret))
		goto irq_err;

	tsp_verb("intr msg: 0x%08x\n", *msg);

	if (unlikely(*msg == 0xE11CE970))     // TSP IC Exception
		goto irq_ic_err;

	if (unlikely(*msg == 0 || *msg == 0xFFFFFFFF))  // Unknown CMD
		goto irq_err;

	event_ms = ms;
	ist30xx_put_track_ms(event_ms);
	ist30xx_put_track(&tracking_intr_value, 1);
	ist30xx_put_track(msg, 1);

	if (unlikely((*msg & CALIB_MSG_MASK) == CALIB_MSG_VALID)) {
		data->status.calib_msg = *msg;
		tsp_info("calib status: 0x%08x\n", data->status.calib_msg);

		goto irq_end;
#if IST30XX_CHKIC
	} else if (*msg == IST30XX_CHKIC_END) {
		data->status.chkic_msg = *msg;
		tsp_info("IC check status: 0x%08x\n", data->status.chkic_msg);

		goto irq_end;
#endif
	}

#if IST30XX_ENABLE_GESTURE
	ret = PARSE_GESTURE_MESSAGE(*msg);
	if (unlikely(ret > 0)) {
		tsp_info("Gesture ID: %d (0x%08x)\n", ret, *msg);
		ist30xx_gesture_cmd(data, ret);

		goto irq_end;
	}
#endif
	memset(data->fingers, 0, sizeof(data->fingers));

	/* Unknown interrupt data for extend coordinate */
	if (unlikely(!CHECK_INTR_STATUS(*msg)))
		goto irq_err;

	data->t_status = *msg;
	key_cnt = PARSE_KEY_CNT(data->t_status);
	finger_cnt = PARSE_FINGER_CNT(data->t_status);
	read_cnt = finger_cnt;

	if (unlikely((finger_cnt > data->max_fingers) ||
		     (key_cnt > data->max_keys))) {
		tsp_warn("Invalid touch count - finger: %d(%d), key: %d(%d)\n",
			 finger_cnt, data->max_fingers, key_cnt, data->max_keys);
		goto irq_err;
	}

	if (read_cnt > 0) {
#if I2C_BURST_MODE
		ret = ist30xx_get_position(data->client, &msg[offset], read_cnt);
		if (unlikely(ret))
			goto irq_err;

		for (i = 0; i < read_cnt; i++)
			data->fingers[i].full_field = msg[i + offset];
#else
		for (i = 0; i < read_cnt; i++) {
			ret = ist30xx_get_position(data->client, &msg[i + offset], 1);
			if (unlikely(ret))
				goto irq_err;

			data->fingers[i].full_field = msg[i + offset];
		}
#endif          // I2C_BURST_MODE

		ist30xx_put_track(msg + offset, read_cnt);
		for (i = 0; i < read_cnt; i++)
			tsp_verb("intr msg(%d): 0x%08x\n", i + offset, msg[i + offset]);
	}

	ret = check_valid_coord(&msg[0], read_cnt + 1);
	if (unlikely(ret < 0))
		goto irq_err;

	if (unlikely(check_report_fingers(data, finger_cnt)))
		goto irq_end;

	report_input_data(data, finger_cnt, key_cnt);

	if (intr_debug2_addr > 0 && intr_debug2_size > 0) {
		tsp_notc("Intr_debug2 (addr: 0x%08x)\n", intr_debug2_addr);
		for (i = 0; i < intr_debug2_size; i++) {
			ist30xx_read_buf(data->client,
				intr_debug2_addr + IST30XX_DATA_LEN * i, &msg[i], 1);
			tsp_notc("\t%08x\n", msg[i]);
		}
		ist30xx_put_track(msg, intr_debug2_size);
	}

	if (intr_debug3_addr > 0 && intr_debug3_size > 0) {
		tsp_notc("Intr_debug3 (addr: 0x%08x)\n", intr_debug3_addr);
		for (i = 0; i < intr_debug3_size; i++) {
			ist30xx_read_buf(data->client,
				intr_debug3_addr + IST30XX_DATA_LEN * i, &msg[i], 1);
			tsp_notc("\t%08x\n", msg[i]);
		}
		ist30xx_put_track(msg, intr_debug3_size);
	}

	goto irq_end;

irq_err:
	tsp_err("intr msg: 0x%08x, ret: %d\n", msg[0], ret);
	ist30xx_request_reset();
irq_end:
	ist30xx_irq_working = false;
	event_ms = (u32)get_milli_second();
	return IRQ_HANDLED;

irq_ic_err:
	tsp_err("Occured IC exception\n");
	ist30xx_scheduled_reset();
	ist30xx_irq_working = false;
	event_ms = (u32)get_milli_second();
	return IRQ_HANDLED;
}

static int ist30xx_pinctrl_configure(struct ist30xx_data *data, bool active);

#ifdef CONFIG_HAS_EARLYSUSPEND
#define ist30xx_suspend NULL
#define ist30xx_resume  NULL
static void ist30xx_early_suspend(struct early_suspend *h)
{
	struct ist30xx_data *data = container_of(h, struct ist30xx_data,
						 early_suspend);

	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);
	del_timer(&event_timer);
	cancel_delayed_work_sync(&work_noise_protect);
	cancel_delayed_work_sync(&work_reset_check);
	cancel_delayed_work_sync(&work_debug_algorithm);
	ist30xx_internal_suspend(data);
	clear_input_data(data);
#if IST30XX_ENABLE_GESTURE
	ist30xx_start(data);
	data->status.noise_mode = false;
	ist30xx_enable_irq(data);
#endif
	mutex_unlock(&ist30xx_mutex);
}
static void ist30xx_late_resume(struct early_suspend *h)
{
	struct ist30xx_data *data = container_of(h, struct ist30xx_data,
						 early_suspend);

	ist30xx_power_status = 1;

	mutex_lock(&ist30xx_mutex);
	ist30xx_internal_resume(data);
	ist30xx_start(data);
	ist30xx_enable_irq(data);
	mutex_unlock(&ist30xx_mutex);
}
#else
static int ist30xx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ist30xx_data *data = i2c_get_clientdata(client);

	mutex_lock(&ist30xx_mutex);

	ist30xx_disable_irq(data);
	del_timer(&event_timer);
	cancel_delayed_work_sync(&work_noise_protect);
	cancel_delayed_work_sync(&work_reset_check);
	cancel_delayed_work_sync(&work_debug_algorithm);

	ist30xx_internal_suspend(data);
	clear_input_data(data);
	ist30xx_pinctrl_configure(data, false);

#if IST30XX_ENABLE_GESTURE
	ist30xx_start(data);
	data->status.noise_mode = false;
	ist30xx_enable_irq(data);
#endif

	mutex_unlock(&ist30xx_mutex);

	return 0;
}
static int ist30xx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ist30xx_data *data = i2c_get_clientdata(client);

	ist30xx_power_status = 1;

	mutex_lock(&ist30xx_mutex);

	ist30xx_pinctrl_configure(data, true);
	ist30xx_internal_resume(data);
	ist30xx_start(data);
	ist30xx_enable_irq(data);

	mutex_unlock(&ist30xx_mutex);

	return 0;
}
#endif // CONFIG_HAS_EARLYSUSPEND

#ifdef USE_OPEN_CLOSE
static void  ist30xx_ts_close(struct input_dev *dev)
{
	struct ist30xx_data *data = input_get_drvdata(dev);

	if (data->touch_stopped) {
		tsp_err("%s: IC is already on\n", __func__);
		return;
	}

	tsp_info("%s\n", __func__);
	data->touch_stopped = true;
	ist30xx_suspend(&data->client->dev);

}
static int ist30xx_ts_open(struct input_dev *dev)
{
	struct ist30xx_data *data = input_get_drvdata(dev);

	if (!data->touch_stopped) {
		tsp_err("%s: IC is already on\n", __func__);
		return 0;
	}
	tsp_info("%s\n", __func__);

	ist30xx_resume(&data->client->dev);
	data->touch_stopped = false;

	tsp_info("%s: IC version core: %x, param: %x, sub: %x\n", __func__,
		 data->fw.core_ver, data->fw.param_ver, data->fw.sub_ver);

	return 0;
}
#endif

void ist30xx_set_ta_mode(bool charging)
{
	tsp_info("%s(), charging = %d\n", __func__, charging);

	if (unlikely(charging == ist30xx_ta_status))
		return;

	if (unlikely(ist30xx_noise_mode == -1)) {
		ist30xx_ta_status = charging ? 1 : 0;
		return;
	}

	ist30xx_ta_status = charging ? 1 : 0;

	ist30xx_scheduled_reset();
}
EXPORT_SYMBOL(ist30xx_set_ta_mode);

void ist30xx_set_call_mode(int mode)
{
	tsp_info("%s(), mode = %d\n", __func__, mode);

	if (unlikely(mode == ((ist30xx_noise_mode >> NOISE_MODE_CALL) & 1)))
		return;

	ist30xx_noise_mode &= ~(1 << NOISE_MODE_CALL);
	if (mode)
		ist30xx_noise_mode |= (1 << NOISE_MODE_CALL);

	ist30xx_scheduled_reset();
}
EXPORT_SYMBOL(ist30xx_set_call_mode);

void ist30xx_set_cover_mode(int mode)
{
	tsp_info("%s(), mode = %d\n", __func__, mode);

	if (unlikely(mode == ((ist30xx_noise_mode >> NOISE_MODE_COVER) & 1)))
		return;

	ist30xx_noise_mode &= ~(1 << NOISE_MODE_COVER);
	if (mode)
		ist30xx_noise_mode |= (1 << NOISE_MODE_COVER);

	ist30xx_scheduled_reset();
}
EXPORT_SYMBOL(ist30xx_set_cover_mode);

#ifdef USE_TSP_TA_CALLBACKS

void charger_enable(struct tsp_callbacks *cb, int enable)
{
	bool charging = enable ? true : false;

	ist30xx_set_ta_mode(charging);
}

static void ist30xx_register_callback(struct tsp_callbacks *cb)
{
	charger_callbacks = cb;
	pr_info("%s\n", __func__);
}

#else
void charger_enable(int enable)
{
	bool charging = enable ? true : false;

	ist30xx_set_ta_mode(charging);
}
#endif

static void reset_work_func(struct work_struct *work)
{
	if (unlikely((ts_data == NULL) || (ts_data->client == NULL)))
		return;

	tsp_info("Request reset function\n");

	if (likely((ist30xx_initialized == 1) && (ts_data->status.power == 1) &&
		   (ts_data->status.update != 1) && (ts_data->status.calib != 1))) {
		mutex_lock(&ist30xx_mutex);
		ist30xx_disable_irq(ts_data);
		clear_input_data(ts_data);
		ist30xx_reset(false);
		ist30xx_start(ts_data);
		ist30xx_enable_irq(ts_data);
		mutex_unlock(&ist30xx_mutex);
	}
}

#if IST30XX_INTERNAL_BIN && IST30XX_UPDATE_BY_WORKQUEUE
static void fw_update_func(struct work_struct *work)
{
	if (unlikely((ts_data == NULL) || (ts_data->client == NULL)))
		return;

	tsp_info("FW update function\n");

	if (likely(ist30xx_auto_bin_update(ts_data)))
		ist30xx_disable_irq(ts_data);
}
#endif // IST30XX_INTERNAL_BIN && IST30XX_UPDATE_BY_WORKQUEUE


#if IST30XX_EVENT_MODE
u32 ist30xx_max_scan_retry = 2;
u32 ist30xx_algr_addr = 0, ist30xx_algr_size = 0;

#define SCAN_STATUS_MAGIC       (0x3C000000)
#define SCAN_STATUS_MASK        (0xFF000000)
#define FINGER_CNT_MASK         (0x00F00000)
#define SCAN_CNT_MASK           (0x000FFFFF)
#define GET_FINGER_CNT(n)       ((n & FINGER_CNT_MASK) >> 20)
#define GET_SCAN_CNT(n)         (n & SCAN_CNT_MASK)
static void noise_work_func(struct work_struct *work)
{
#if IST30XX_NOISE_MODE
	int ret;
	u32 scan_status = 0;

	ret = ist30xx_read_cmd(ts_data->client, IST30XXB_MEM_COUNT, &scan_status);
	if (unlikely(ret)) {
		tsp_warn("Mem scan count read fail!\n");
		goto retry_timer;
	}

	ist30xx_put_track_ms(timer_ms);
	ist30xx_put_track(&scan_status, 1);

	tsp_verb("scan status: 0x%x\n", scan_status);

	/* Check valid scan count */
	if (unlikely((scan_status & SCAN_STATUS_MASK) != SCAN_STATUS_MAGIC)) {
		tsp_warn("Scan status is not corrected! (0x%08x)\n", scan_status);
		goto retry_timer;
	}

	/* Status of IC is idle */
	if (GET_FINGER_CNT(scan_status) == 0) {
		if ((PARSE_FINGER_CNT(ts_data->t_status) > 0) ||
		    (PARSE_KEY_CNT(ts_data->t_status) > 0))
			clear_input_data(ts_data);
	}

	scan_status &= SCAN_CNT_MASK;

	/* Status of IC is lock-up */
	if (unlikely(scan_status == ist30xx_scan_count)) {
		tsp_warn("TSP IC is not responded! (0x%08x)\n", scan_status);
		goto retry_timer;
	} else {
		ist30xx_scan_retry = 0;
	}

	ist30xx_scan_count = scan_status;

	return;

retry_timer:
	ist30xx_scan_retry++;
	tsp_warn("Retry scan status!(%d)\n", ist30xx_scan_retry);

	if (unlikely(ist30xx_scan_retry == ist30xx_max_scan_retry)) {
		ist30xx_scheduled_reset();
		ist30xx_scan_retry = 0;
	}
#endif  // IST30XX_NOISE_MODE
}

static void debug_work_func(struct work_struct *work)
{
#if IST30XX_ALGORITHM_MODE
	int ret = -EPERM;
	int i;
	u32 *buf32;

	buf32 = kzalloc(ist30xx_algr_size, GFP_KERNEL);

	for (i = 0; i < ist30xx_algr_size; i++) {
		ret = ist30xx_read_buf(ts_data->client,
			ist30xx_algr_addr + IST30XX_DATA_LEN * i, &buf32[i], 1);
		if (ret) {
			tsp_warn("Algorithm mem addr read fail!\n");
			return;
		}
	}

	ist30xx_put_track(buf32, ist30xx_algr_size);

	tsp_debug(" 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
		buf32[0], buf32[1], buf32[2], buf32[3], buf32[4]);
#endif
}

void timer_handler(unsigned long data)
{
	struct ist30xx_status *status = &ts_data->status;

	if (ist30xx_irq_working)
		goto restart_timer;

	if (status->event_mode) {
		if (likely((status->power == 1) && (status->update != 1))) {
			timer_ms = (u32)get_milli_second();
			if (unlikely(status->calib == 1)) { // Check calibration
				if ((status->calib_msg & CALIB_MSG_MASK) == CALIB_MSG_VALID) {
					tsp_info("Calibration check OK!!\n");
					schedule_delayed_work(&work_reset_check, 0);
					status->calib = 0;
				} else if (timer_ms - event_ms >= 3000) { // 3second
					tsp_info("calibration timeout over 3sec\n");
					schedule_delayed_work(&work_reset_check, 0);
					status->calib = 0;
				}
			} else if (likely(status->noise_mode)) {
				if (timer_ms - event_ms > 100)     // 100ms after last interrupt
					schedule_delayed_work(&work_noise_protect, 0);
			}

#if IST30XX_ALGORITHM_MODE
			if ((ist30xx_algr_addr >= IST30XXB_ACCESS_ADDR) &&
			    (ist30xx_algr_size > 0)) {
				if (timer_ms - event_ms > 100)     // 100ms after last interrupt
					schedule_delayed_work(&work_debug_algorithm, 0);
			}
#endif
		}
	}

restart_timer:
	mod_timer(&event_timer, get_jiffies_64() + EVENT_TIMER_INTERVAL);
}
#endif // IST30XX_EVENT_MODE

static int ist30xx_pinctrl_configure(struct ist30xx_data *data, bool active)
{
	struct pinctrl_state *set_state_en;
	int retval;

	if (!data->pinctrl)
		return 0;

	tsp_err("%s: %s\n", __func__, active ? "ACTIVE" : "SUSPEND");

	if (active) {
		set_state_en =
			pinctrl_lookup_state(data->pinctrl,
						"tsp_en_gpio_active");
		if (IS_ERR(set_state_en)) {
			tsp_err("%s: cannot get pinctrl(tsp_en) active state\n", __func__);
			return PTR_ERR(set_state_en);
		}
	} else {
		set_state_en =
			pinctrl_lookup_state(data->pinctrl,
						"tsp_en_gpio_suspend");
		if (IS_ERR(set_state_en)) {
			tsp_err("%s: cannot get pinctrl(tsp_en) sleep state\n", __func__);
			return PTR_ERR(set_state_en);
		}
	}

	retval = pinctrl_select_state(data->pinctrl, set_state_en);
	if (retval) {
		tsp_err("%s: cannot set pinctrl(tsp_en) %s state\n",
				__func__, active ? "active" : "suspend");
		return retval;
	}

	return 0;
}

static void ist30xx_request_gpio(struct i2c_client *client,
				 struct ist30xx_data *data)
{
	int ret;

	tsp_info("%s\n", __func__);

	if (gpio_is_valid(data->dt_data->touch_en_gpio)) {
		ret = gpio_request(data->dt_data->touch_en_gpio,
				   "imagis,touch_en_gpio");
		if (ret) {
			tsp_err("%s: unable to request touch_en_gpio [%d]\n",
				__func__, data->dt_data->touch_en_gpio);
			return;
		}
	}
#if SUPPORTED_TOUCH_KEY_LED
	if (gpio_is_valid(data->dt_data->keyled_en_gpio)) {
		ret = gpio_request(data->dt_data->keyled_en_gpio,
				   "imagis,keyled_en_gpio");
		if (ret) {
			tsp_err("%s: unable to request keyled_en_gpio [%d]\n",
				__func__, data->dt_data->keyled_en_gpio);
			return;
		}
	}
#endif
	if (gpio_is_valid(data->dt_data->irq_gpio)) {
		ret = gpio_request(data->dt_data->irq_gpio, "imagis,irq_gpio");
		if (ret) {
			tsp_err("%s: unable to request irq_gpio [%d]\n",
				__func__, data->dt_data->irq_gpio);
			return;
		}

		ret = gpio_direction_input(data->dt_data->irq_gpio);
		if (ret) {
			tsp_err("%s: unable to set direction for gpio [%d]\n",
				__func__, data->dt_data->irq_gpio);
		}
		client->irq = gpio_to_irq(data->dt_data->irq_gpio);
	}
}

static void ist30xx_free_gpio(struct ist30xx_data *data)
{
	tsp_info("%s\n", __func__);

	if (gpio_is_valid(data->dt_data->touch_en_gpio))
		gpio_free(data->dt_data->touch_en_gpio);

	if (gpio_is_valid(data->dt_data->irq_gpio))
		gpio_free(data->dt_data->irq_gpio);
#if SUPPORTED_TOUCH_KEY_LED
	if (gpio_is_valid(data->dt_data->keyled_en_gpio))
		gpio_free(data->dt_data->keyled_en_gpio);
#endif
}

#ifdef CONFIG_OF
static int ist30xx_parse_dt(struct device *dev, struct ist30xx_data *data)
{
	struct device_node *np = dev->of_node;
	int ret;

	data->dt_data->irq_gpio = of_get_named_gpio(np, "imagis,irq-gpio", 0);
	data->dt_data->touch_en_gpio = of_get_named_gpio(np, "vdd_en-gpio", 0);
#if SUPPORTED_TOUCH_KEY_LED
	data->dt_data->keyled_en_gpio = of_get_named_gpio(np, "keyled_en-gpio", 0);
#endif

	ret = of_property_read_string(np, "imagis,pname", &data->dt_data->pname);
	if (ret < 0) {
		pr_err("%s: failed to read property config_ver\n",
			__func__);
		data->dt_data->pname = "SEC";
	}

	tsp_info("%s() irq:%d, touch_en:%d, keyled_en:%d, project:%s\n",
		 __func__, data->dt_data->irq_gpio, data->dt_data->touch_en_gpio,
		 data->dt_data->keyled_en_gpio, data->dt_data->pname);

	return 0;
}
#else
static int ist30xx_parse_dt(struct device *dev, struct ist30xx_data *data)
{
	return -ENODEV;
}
#endif

static int ist30xx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	int retry = 3;
	struct ist30xx_data *data;
	struct input_dev *input_dev;

	tsp_info("##### IST30XX(Ver%d) Initialize #####\n", IST30XX_DD_VERSION);
	tsp_info("%s(), the i2c addr=0x%x\n", __func__, client->addr);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ts_data = data;
	data->dt_data = NULL;
	data->irq_enabled = 1;
	data->client = client;
	i2c_set_clientdata(client, data);

	if (client->dev.of_node) {
		data->dt_data = kzalloc(sizeof(struct ist30xx_dt_data), GFP_KERNEL);

		if (!data->dt_data)
			goto err_alloc_dev;

		ret = ist30xx_parse_dt(&client->dev, data);
		if (ret)
			goto err_alloc_dt;
	} else {
		data->dt_data = NULL;
		tsp_err("%s: TSP failed to align dtsi\n", __func__);
	}

	if (data->dt_data)
		ist30xx_request_gpio(client, data);

	/* Get pinctrl if target uses pinctrl */
	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(data->pinctrl)) {
		if (PTR_ERR(data->pinctrl) == -EPROBE_DEFER)
			goto err_pinctrl;

		tsp_err("%s: Target does not use pinctrl\n", __func__);
		data->pinctrl = NULL;
	}

	ret = ist30xx_pinctrl_configure(data, true);
	if (ret)
		tsp_err("%s: cannot set pinctrl state\n", __func__);

	input_dev = input_allocate_device();
	if (unlikely(!input_dev)) {
		ret = -ENOMEM;
		tsp_err("%s(), input_allocate_device failed (%d)\n",
			__func__, ret);
		goto err_alloc_dev;
	}

	data->max_fingers = data->max_keys = IST30XX_MAX_MT_FINGERS;
	data->input_dev = input_dev;

	input_mt_init_slots(input_dev, IST30XX_MAX_MT_FINGERS, 0);

	input_dev->name = "sec_touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
#if defined(USE_OPEN_CLOSE)
	input_dev->open = ist30xx_ts_open;
	input_dev->close = ist30xx_ts_close;
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#if SUPPORTED_TOUCH_KEY_LED
	set_bit(EV_LED, input_dev->evbit);
	set_bit(LED_MISC, input_dev->ledbit);
#endif
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, IST30XX_MAX_X - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, IST30XX_MAX_Y - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, IST30XX_MAX_W, 0, 0);

#if IST30XX_USE_KEY
	{
		int i;
		set_bit(EV_KEY, input_dev->evbit);
		set_bit(EV_SYN, input_dev->evbit);
		for (i = 1; i < ARRAY_SIZE(ist30xx_key_code); i++)
			set_bit(ist30xx_key_code[i], input_dev->keybit);
	}
#if IST30XX_ENABLE_GESTURE
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_PLAYPAUSE);
	input_set_capability(input_dev, EV_KEY, KEY_NEXTSONG);
	input_set_capability(input_dev, EV_KEY, KEY_PREVIOUSSONG);
	input_set_capability(input_dev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(input_dev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(input_dev, EV_KEY, KEY_MUTE);
	input_set_capability(input_dev, EV_KEY, KEY_SCREENLOCK);
	input_set_capability(input_dev, EV_KEY, KEY_LOGOFF);
#endif
#endif

	input_set_drvdata(input_dev, data);
	ret = input_register_device(input_dev);
	if (unlikely(ret)) {
		input_free_device(input_dev);
		goto err_reg_dev;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = ist30xx_early_suspend;
	data->early_suspend.resume = ist30xx_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	ret = ist30xx_init_system();
	if (unlikely(ret)) {
		tsp_err("chip initialization failed\n");
		goto err_init_drv;
	}

	ret = ist30xx_init_update_sysfs();
	if (unlikely(ret))
		goto err_init_drv;

#if IST30XX_DEBUG
	ret = ist30xx_init_misc_sysfs();
	if (unlikely(ret))
		goto err_init_drv;
#endif

#if IST30XX_CMCS_TEST
	ret = ist30xx_init_cmcs_sysfs();
	if (unlikely(ret))
		goto err_init_drv;
#endif

#if SEC_FACTORY_MODE
	ret = sec_fac_cmd_init(data);
	if (unlikely(ret))
		goto err_init_drv;
	ret = sec_touch_sysfs(data);
	if (unlikely(ret))
		goto err_init_drv;
#endif

#if IST30XX_TRACKING_MODE
	ret = ist30xx_init_tracking_sysfs();
	if (unlikely(ret))
		goto err_init_drv;
#endif

	ret = request_threaded_irq(client->irq, NULL, ist30xx_irq_thread,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ist30xx_ts", data);
	if (unlikely(ret))
		goto err_irq;

	ist30xx_disable_irq(data);

	while ((data->chip_id != IST30XXB_CHIP_ID) &&
	       (data->chip_id != IST3038_CHIP_ID)) {
		ret = ist30xx_read_cmd(data->client, IST30XXB_REG_CHIPID,
				       &data->chip_id);
		if (unlikely(ret)) {
			tsp_info("tspid: %x\n", data->chip_id);
			ist30xx_reset(false);
		}

		if (data->chip_id == 0x3000B)
			data->chip_id = IST30XXB_CHIP_ID;

        if (data->chip_id == 0x3038B)
			data->chip_id = IST3038_CHIP_ID;

		if (retry-- == 0)
			break;
	}

	retry = 3;
	while (retry-- > 0) {
		ret = ist30xx_read_cmd(data->client, IST30XXB_REG_TSPTYPE,
				       &data->tsp_type);

		tsp_info("tsptype: %x\n", data->tsp_type);
		data->tsp_type = IST30XXB_PARSE_TSPTYPE(data->tsp_type);

		if (likely(ret == 0))
			break;

		if (unlikely(retry == 0))
			goto err_irq;
	}

	tsp_info("TSP IC: %x, TSP Vendor: %x\n", data->chip_id, data->tsp_type);

	data->status.event_mode = false;

#if IST30XX_INTERNAL_BIN
#if IST30XX_UPDATE_BY_WORKQUEUE
	INIT_DELAYED_WORK(&work_fw_update, fw_update_func);
	schedule_delayed_work(&work_fw_update, IST30XX_UPDATE_DELAY);
#else
	ret = ist30xx_auto_bin_update(data);
	if (unlikely(ret != 0))
		goto err_irq;
#endif
#endif  // IST30XX_INTERNAL_BIN

	if (ist30xx_ta_status < 0)
		ist30xx_ta_status = 0;

	if (ist30xx_power_status < 0)
		ist30xx_power_status = 0;

	if (ist30xx_noise_mode < 0)
		ist30xx_noise_mode = 0;

	ret = ist30xx_get_info(data);
	tsp_info("Get info: %s\n", (ret == 0 ? "success" : "fail"));

	INIT_DELAYED_WORK(&work_reset_check, reset_work_func);
#if IST30XX_EVENT_MODE
	INIT_DELAYED_WORK(&work_noise_protect, noise_work_func);
	INIT_DELAYED_WORK(&work_debug_algorithm, debug_work_func);
#endif

#if IST30XX_EVENT_MODE
	init_timer(&event_timer);
	event_timer.function = timer_handler;
	event_timer.expires = jiffies_64 + (EVENT_TIMER_INTERVAL);
	mod_timer(&event_timer, get_jiffies_64() + EVENT_TIMER_INTERVAL);
#endif

#ifdef USE_TSP_TA_CALLBACKS
	data->register_cb = ist30xx_register_callback;
	data->callbacks.inform_charger = charger_enable;
	if (data->register_cb)
		data->register_cb(&data->callbacks);
#endif

	ist30xx_initialized = 1;

	return 0;

err_irq:
	tsp_info("ChipID: %x\n", data->chip_id);
	ist30xx_disable_irq(data);
	free_irq(client->irq, data);
err_init_drv:
	data->status.event_mode = false;
	tsp_err("Error, ist30xx init driver\n");
	ist30xx_power_off();
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	input_unregister_device(input_dev);
	return 0;

err_reg_dev:
	pinctrl_put(data->pinctrl);
err_pinctrl:
	if (data->dt_data) {
		tsp_err("%s: Error, ist30xx mem free, line:%d\n", __func__, __LINE__);
		kfree(data->dt_data);
	}
err_alloc_dt:
err_alloc_dev:
	tsp_err("Error, ist30xx mem free\n");
	kfree(data);
	return 0;
}


static int ist30xx_remove(struct i2c_client *client)
{
	struct ist30xx_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	ist30xx_disable_irq(data);
	free_irq(client->irq, data);
	ist30xx_power_off();

	input_unregister_device(data->input_dev);

	if (data->dt_data) {
		ist30xx_free_gpio(data);
		kfree(data->dt_data);
	}
	kfree(data);

	return 0;
}


static struct i2c_device_id ist30xx_idtable[] = {
	{ IST30XX_DEV_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ist30xx_idtable);

#ifdef CONFIG_OF
static struct of_device_id ist30xx_match_table[] = {
	{ .compatible = "imagis,ist30xx-ts", },
	{ },
};
#else
#define ist30xx_match_table NULL
#endif

static struct i2c_driver ist30xx_i2c_driver = {
	.id_table	= ist30xx_idtable,
	.probe		= ist30xx_probe,
	.remove		= ist30xx_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= IST30XX_DEV_NAME,
		.of_match_table = ist30xx_match_table,
	},
};

#ifdef CONFIG_SAMSUNG_LPM_MODE
extern int poweroff_charging;
#endif

extern int get_lcd_attached(void);

static int __init ist30xx_init(void)
{
#ifdef CONFIG_SAMSUNG_LPM_MODE
	if (poweroff_charging) {
		tsp_info("%s() LPM Charging Mode!!\n", __func__);
		return 0;
	}
#endif
	if (get_lcd_attached() == 0) {
		tsp_info("%s() LCD is not attached!!\n", __func__);
		return 0;
	}

	return i2c_add_driver(&ist30xx_i2c_driver);
}

static void __exit ist30xx_exit(void)
{
	i2c_del_driver(&ist30xx_i2c_driver);
}

module_init(ist30xx_init);
module_exit(ist30xx_exit);

MODULE_DESCRIPTION("Imagis IST30XX touch driver");
MODULE_LICENSE("GPL");
