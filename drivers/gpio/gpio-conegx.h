/**
 * @file gpio-conegx.h
 * @author A. Pietsch (a.pietsch@consolinno.de)
 * @brief Driver for Consolinno Conegx Module
 * @version 1.0.0
 * @date 2021-06-22
 * 
 * @copyright: Copyrigth (c) 2021
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */


#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/leds.h>

#ifndef __CONEGX_DRIVER
#define __CONEGX_DRIVER

/* DIVER VERSION*/
#define DRIVER_VERSION "1.0.0"


/* CONEGX REGISTERMAP */
#define GET_VOLTAGE_PORT 0x01
#define GET_INPUT_PORT 0x02
#define SET_RELAY_PORT 0x03
#define GET_RELAY_PORT 0x04
#define SET_HBUS_PORT 0x05
#define GET_HBUS_PORT 0x06
#define SET_HBUS_DIRECTION 0x07
#define GET_HBUS_DIRECTION 0x08
#define SET_LED_PORT_0 0x09
#define GET_LED_PORT_0 0x0A
#define SET_LED_PORT_1 0x0B
#define GET_LED_PORT_1 0x0C
#define ALERT 0x0D
#define SET_DEFAULT_PARAMETER 0x0E
#define GET_DEFAULT_PARAMETER 0x0F
#define FW_VERSION_MAJOR 0x010
#define FW_VERSION_MINOR_1 0x11
#define FW_VERSION_MINOR_2 0x12
#define DEVICE_DESCRIPTION 0x13
#define GET_POWER_FAILURE 0x14
#define SET_OS_READY 0x15
#define SET_BUTTON_LOCK 0x16
#define GET_BUTTON_LOCK 0x17

#define WRITE 1
#define READ 0
#define WRITE 1
#define READ 0

/* GPIO NUMBERS */
#define IO_RELAY_1 0
#define IO_RELAY_2 1
#define IO_RELAY_3 2
#define IO_RELAY_4 3
#define IO_MRES_M2 4
#define IO_MRES_M1 5
#define IO_MRES_S2 6
#define IO_MRES_S1 7
#define IO_FLT_HBUS24 8
#define IO_FLT_HBUS 9
#define IO_RST_BUTTON 10
#define IO_TST_BUTTON 11
#define IO_PFI_1 12
#define IO_PFI_2 13
#define IO_PFI_3 14
#define IO_PFI_4 15

/* LED NUMBERS */
#define IO_LED_1 0
#define IO_LED_2 1
#define IO_LED_3 2
#define IO_LED_4 3
#define IO_LED_5 4
#define IO_LED_6 5
#define IO_RGBLED_1_1 6
#define IO_RGBLED_1_2 7
#define IO_RGBLED_1_3 8



/* ------------------------------IRQ------------------------------ */

/* IRQ EDGES */
#define FALLING_EDGE 0
#define RISING_EDGE 1

/* IRQ NUMBERS */

#define POWER_FAILURE_INTERUPT 1
#define VOLTAGE_ALERT_INTERRUPT 2
#define VOLTAGE_NORMAL_INTERRUPT 3
#define	POTENTIAL_FREE_INPUT_1_RISING_EDGE		14
#define	POTENTIAL_FREE_INPUT_1_FALLING_EDGE		15
#define	POTENTIAL_FREE_INPUT_2_RISING_EDGE		16
#define	POTENTIAL_FREE_INPUT_2_FALLING_EDGE		17
#define	POTENTIAL_FREE_INPUT_3_RISING_EDGE		18
#define	POTENTIAL_FREE_INPUT_3_FALLING_EDGE		19
#define	POTENTIAL_FREE_INPUT_4_RISING_EDGE		20
#define	POTENTIAL_FREE_INPUT_4_FALLING_EDGE		21
#define	FLT_HBUS_RISING_EDGE		22
#define	FLT_HBUS_FALLING_EDGE		23
#define	FLT_HBUS24_RISING_EDGE		24
#define	FLT_HBUS24_FALLING_EDGE		25
#define	MRES_M1_RISING_EDGE		26
#define	MRES_M1_FALLING_EDGE		27
#define	MRES_M2_RISING_EDGE		28
#define	MRES_M2_FALLING_EDGE		29
#define	MRES_S1_RISING_EDGE		30
#define	MRES_S1_FALLING_EDGE		31
#define	MRES_S2_RISING_EDGE		32
#define	MRES_S2_FALLING_EDGE		33

/* LED */
#define NR_OF_LEDS 9
#define LED_FULL 255


/**
 * @brief conegx_gpio_irq_map [GpioNr, Edge]
 * @description: maps IRQ Numbers to GPio Pins and Edges 
 */
const int conegx_gpio_irq_map[20][2] = {
	{12, RISING_EDGE},	//	PFI 1
	{12, FALLING_EDGE}, //	PFI 1
	{13, RISING_EDGE},	//	PFI 2
	{13, FALLING_EDGE}, //	PFI 2
	{14, RISING_EDGE},	//	PFI 3
	{14, FALLING_EDGE}, //	PFI 3
	{15, RISING_EDGE},	//	PFI 4
	{15, FALLING_EDGE}, //	PFI 4
	{9, RISING_EDGE},	//	FLT_HBUS
	{9, FALLING_EDGE},	//	FLT_HBUS
	{8, RISING_EDGE},	//	FLT_HBUS_24
	{8, FALLING_EDGE},	//	FLT_HBUS_24
	{5, RISING_EDGE},	//	MRES_M1
	{5, FALLING_EDGE},	//	MRES_M1
	{4, RISING_EDGE},	//	MRES_M2
	{4, FALLING_EDGE},	//	MRES_M2
	{7, RISING_EDGE},	//	MRES_S1
	{7, FALLING_EDGE},	//	MRES_S1
	{6, RISING_EDGE},	//	MRES_S2
	{6, FALLING_EDGE},	//	MRES_S2

};

/**
 * @brief CONEGX REGISTER access 
 * READ= read acces only
 * WRITE=  write acces only
 */
const bool conegx_reg_access[23] = {
	READ,  //	Get Voltage Port
	READ,  //	Get Input Port
	WRITE, //	Set Relay Port
	READ,  //	Get Relay Port
	WRITE, //	Set HBUS Port
	READ,  //	Get HBUS Port
	WRITE, //	Set HBUS Direction
	READ,  //	Get HBUS Direction
	WRITE, //	Set LED Port 0
	READ,  //	Get LED Port 0
	WRITE, //	Set LED Port 1
	READ,  //	Get LED Port 1
	READ,  //	Alert
	WRITE, //	Set Default Parameter
	READ,  //	Get Default Parameter
	READ,  //	Get FW Version Major
	READ,  //	Get FW Version Minor 1
	READ,  //	Get FW Version Minor 2
	READ,  //	Get Device Description
	READ,  //	Get Power Failure
	WRITE, //	Set OS READy
	WRITE, //	Set Button Lock
	READ,  //	Get Button Lock

};

const char *const conegx_gpio_names[] = {

	"S_1",
	"S_2",
	"W_3",
	"W_4",
	"MRES_M2",
	"MRES_M1",
	"MRES_S2",
	"MRES_S1",
	"FLT_HBUS24",
	"FLT_HBUS",
	"RST_Butt",
	"TST_Butt",
	"SwitchIN_1",
	"SwitchIN_2",
	"SwitchIN_3",
	"SwitchIN_4",
};

const char *const conegx_led_names[] = {

	"CON:LED1",
	"CON:LED2",
	"CON:LED3",
	"CON:LED4",
	"CON:LED5",
	"CON:LED6",
	"CON:RGBLED1.1",
	"CON:RGBLED1.2",
	"CON:RGBLED1.3",

};
/**
 * @brief Struct For Conegx LEDs
 * 
 */
struct conegx_led {
	u8 id;
	bool active;
	struct i2c_client *client;
	const char *name;
	const char *default_trigger;
	struct led_classdev ldev;
	struct work_struct work;
	u32 type;
	unsigned int led_no;

};

/**
 * @brief Struct For conegx Data
 * 
 */
struct conegx
{
	struct gpio_chip chip;
	struct irq_chip irq_chip;
	struct mutex lock;
	struct device *dev;
	struct regmap *regmap;
	struct conegx_led leds[NR_OF_LEDS];
	int irq;
	__u8 out;
	__u8 addr;

	/* Register Buffers */
	__u8 SetRelayBuffer;
	__u8 SetHbusBuffer;
	__u8 SetHbusDirectionBuffer;
	__u8 SetLedPort0Buffer;
	__u8 SetLedPort1Buffer;

	/* Device Status Info */
	uint LastInterruptNr;
	int PowerFail;
	int VoltageRange;
	char FwVersion[12];
	int RelayDefaultSetting;
	int TstButtonLock;
	int RstButtonLock;
	int IRQDeviceFileEnabled;
	int IRQVoltageRangeEnabled;
	int IRQPowerFailEnabled;
};


#endif // CONEGX_DRIVER
