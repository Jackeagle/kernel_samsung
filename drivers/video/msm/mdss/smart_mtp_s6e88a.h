/*
 * =================================================================
 *
 *       Filename:  smart_mtp_s6e88a.h
 *
 *    Description:  Smart dimming algorithm implementation
 *
 *        Author: jb09.kim
 *        Company:  Samsung Electronics
 *
 * ================================================================
 */
/*
<one line to give the program's name and a brief idea of what it does.>
Copyright (C) 2012, Samsung Electronics. All rights reserved.

*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#ifndef _SMART_MTP_SE6E8FA_H_
#define _SMART_MTP_SE6E8FA_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/ctype.h>
#include <asm/div64.h>

/*for Kleos*/
#define EVT0_S6E88A0_REV_A 0x01
#define EVT0_S6E88A0_REV_C 0x02
#define EVT0_S6E88A0_REV_D 0x03

/*for Heat*/
#define EVT0_S6E88A0_REV_G 0x03

/*for A3 */
#define AMS452EF_REV_A 0x00
#define AMS452EF_REV_C 0x01
/*
*	From 4.27 inch model use AID function
*	CASE#1 is used for now.
*/
#define AID_OPERATION

#define GAMMA_CURVE_2P25 1
#define GAMMA_CURVE_2P2 2
#define GAMMA_CURVE_2P15 3
#define GAMMA_CURVE_2P1 4
#define GAMMA_CURVE_2P0 5
#define GAMMA_CURVE_1P9 6
#define GAMMA_CURVE_2P2_360 7


#define MTP_START_ADDR 0xC8
#define LUMINANCE_MAX 62
#define GAMMA_SET_MAX 33
#define BIT_SHIFT 22
/*
	it means BIT_SHIFT is 22.  pow(2,BIT_SHIFT) is 4194304.
	BIT_SHIFT is used for right bit shfit
*/
#define BIT_SHFIT_MUL 4194304

#define S6E88A_GRAY_SCALE_MAX 256

/*6.1*4194304 */
#define S6E88A_VREG0_REF 25585254

#define AMS452EF_VREG0_REF_REV_A 24326963 /* 5.8 */
#define AMS452EF_VREG0_REF_REV_C 23488102 /* 5.6 */
/*V0,V1,V3,V11,V23,V35,V51,V87,V151,V203,V255*/
#define S6E88A_MAX 11

/* PANEL DEPENDENT THINGS */
#define MAX_CANDELA 350
#define MIN_CANDELA 5

/*
*	ID 0x20
*/
#define V255_300CD_R_MSB_20 0x01
#define V255_300CD_R_LSB_20 0x00

#define V255_300CD_G_MSB_20 0x01
#define V255_300CD_G_LSB_20 0x00

#define V255_300CD_B_MSB_20 0x01
#define V255_300CD_B_LSB_20 0x00

#define V203_300CD_R_20 0x80
#define V203_300CD_G_20 0x80
#define V203_300CD_B_20 0x80

#define V151_300CD_R_20 0x80
#define V151_300CD_G_20 0x80
#define V151_300CD_B_20 0x80

#define V87_300CD_R_20 0x80
#define V87_300CD_G_20 0x80
#define V87_300CD_B_20 0x80

#define V51_300CD_R_20 0x80
#define V51_300CD_G_20 0x80
#define V51_300CD_B_20 0x80

#define V35_300CD_R_20 0x80
#define V35_300CD_G_20 0x80
#define V35_300CD_B_20 0x80

#define V23_300CD_R_20 0x80
#define V23_300CD_G_20 0x80
#define V23_300CD_B_20 0x80

#define V11_300CD_R_20 0x80
#define V11_300CD_G_20 0x80
#define V11_300CD_B_20 0x80

#if defined(CONFIG_SEC_A3_PROJECT) || defined(CONFIG_SEC_A3_EUR_PROJECT) || defined(CONFIG_SEC_A33G_EUR_PROJECT)
#define V3_300CD_R_20 0x6B
#define V3_300CD_G_20 0x68
#define V3_300CD_B_20 0x71
#else
#define V3_300CD_R_20 0x80
#define V3_300CD_G_20 0x80
#define V3_300CD_B_20 0x80
#endif
#define VT_300CD_R_20 0x00
#define VT_300CD_G_20 0x00
#define VT_300CD_B_20 0x00


/* PANEL DEPENDENT THINGS END*/

enum {
	V1_INDEX = 0,
	V3_INDEX = 1,
	V11_INDEX = 2,
	V23_INDEX = 3,
	V35_INDEX = 4,
	V51_INDEX = 5,
	V87_INDEX = 6,
	V151_INDEX = 7,
	V203_INDEX = 8,
	V255_INDEX = 9,
};

struct GAMMA_LEVEL {
	int level_0;
	int level_1;
	int level_3;
	int level_11;
	int level_23;
	int level_35;
	int level_51;
	int level_87;
	int level_151;
	int level_203;
	int level_255;
} __packed;

struct RGB_OUTPUT_VOLTARE {
	struct GAMMA_LEVEL R_VOLTAGE;
	struct GAMMA_LEVEL G_VOLTAGE;
	struct GAMMA_LEVEL B_VOLTAGE;
} __packed;

struct GRAY_VOLTAGE {
	/*
		This voltage value use 14bit right shit
		it means voltage is divied by 16384.
	*/
	int R_Gray;
	int G_Gray;
	int B_Gray;
} __packed;

struct GRAY_SCALE {
	struct GRAY_VOLTAGE TABLE[S6E88A_GRAY_SCALE_MAX];
	struct GRAY_VOLTAGE VT_TABLE;
} __packed;

/*V0,V1,V3,V11,V23,V35,V51,V87,V151,V203,V255*/

struct MTP_SET {
	char OFFSET_255_MSB;
	char OFFSET_255_LSB;
	char OFFSET_203;
	char OFFSET_151;
	char OFFSET_87;
	char OFFSET_51;
	char OFFSET_35;
	char OFFSET_23;
	char OFFSET_11;
	char OFFSET_3;
	char OFFSET_1;
} __packed;

struct HBM_REG {
	/* LSI */
	char b5_reg[16];	/*B5 13~28th reg*/
	char b6_reg[12];	/*B6 3~14th reg*/
	char b5_reg_19;		/*B5 19th reg*/
	char b6_reg_17; 	/*B6 17th reg*/
	/* MAGNA */
} __packed;

struct MTP_OFFSET {
	struct MTP_SET R_OFFSET;
	struct MTP_SET G_OFFSET;
	struct MTP_SET B_OFFSET;
} __packed;

struct illuminance_table {
	int lux;
	char gamma_setting[GAMMA_SET_MAX];
} __packed;

struct SMART_DIM {
	struct HBM_REG hbm_reg;
	struct MTP_OFFSET MTP_ORIGN;

	struct MTP_OFFSET MTP;
	struct RGB_OUTPUT_VOLTARE RGB_OUTPUT;
	struct GRAY_SCALE GRAY;

	/* Because of AID funtion, below members are added*/
	int lux_table_max;
	int *plux_table;
	struct illuminance_table gen_table[LUMINANCE_MAX];

	int brightness_level;
	int ldi_revision;
	int vregout_voltage;
} __packed;
char* get_b5_reg(void);
char get_b5_reg_19(void);
char* get_b6_reg(void);
char get_b6_reg_17(void);

void get_min_lux_table(char *str, int size);
int smart_dimming_init(struct SMART_DIM *psmart);
void generate_gamma(struct SMART_DIM *smart_dim, char *str, int size);
#endif

