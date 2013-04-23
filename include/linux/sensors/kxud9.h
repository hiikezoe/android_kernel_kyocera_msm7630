#ifndef KXUD9_H
#define KXUD9_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 */
/*
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

#define I2C_SLAVE_WRITE_ADDR (0x30 >> 1)

#define KXUD9_XOUT_H_REG             0x00
#define KXUD9_XOUT_L_REG             0x01
#define KXUD9_YOUT_H_REG             0x02
#define KXUD9_YOUT_L_REG             0x03
#define KXUD9_ZOUT_H_REG             0x04
#define KXUD9_ZOUT_L_REG             0x05
#define KXUD9_AUXOUT_H_REG           0x06
#define KXUD9_AUXOUT_L_REG           0x07

#define KXUD9_RESET_WRITE_REG        0x0a
#define SOFTWARE_POWER_UP_RESET_CODE 0xca

#define KXUD9_CTRL_REGC_REG          0x0c
#define FS0                          (1 << 0)
#define FS1                          (1 << 1)
#define LP0                          (1 << 5)
#define LP1                          (1 << 6)
#define LP2                          (1 << 7)

#define CONFIG_8G_RANGE              (        0)
#define CONFIG_6G_RANGE              (      FS0)
#define CONFIG_4G_RANGE              (FS1      )
#define CONFIG_2G_RANGE              (FS1 + FS0)

#define CONFIG_2000HZ_FREQUENCY      (            LP0)
#define CONFIG_1000HZ_FREQUENCY      (LP2            )
#define CONFIG_500HZ_FREQUENCY       (LP2       + LP0)
#define CONFIG_100HZ_FREQUENCY       (LP2 + LP1      )
#define CONFIG_50HZ_FREQUENCY        (LP2 + LP1 + LP0)

#define KXUD9_CTRL_REGB_REG          0x0d
#define ST                           (1 << 5)
#define ENABLE                       (1 << 6)
#define CLKHLD                       (1 << 7)

#endif
