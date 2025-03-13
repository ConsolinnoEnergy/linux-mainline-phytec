/**
 * @file gpio-conegx.h
 * @author A. Pietsch (a.pietsch@consolinno.de)
 * @brief Driver for Consolinno Conegx Module
 * @version 1.2.1
 * @date 2021-06-22
 * 
 * @copyright: Copyrigth (c) 2021-2024
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
#define DRIVER_VERSION "1.2.1"

/* Conegx Pins */
/** 
 * GPIO number = (bank - 1) * 32 + bit
 * 
 * Reset Pin is GPIO3_IO2 on the i.MX6
 * 
 * => RST_PIN = (3 - 1) * 32 + 2 = 66 
 */
#define RST_PIN 66

#define FW_VERSION_STRING_SIZE 12

/* CONEGX REGISTERMAP */
enum Conegx_Registermap {
DEVICE_DESCRIPTION 	  ,
FW_VERSION_MAJOR 	  ,
FW_VERSION_MINOR_1 	  ,
FW_VERSION_MINOR_2 	  ,
SET_OS_READY 		  ,
GET_INPUT_PORT 	      ,
SET_RELAY_PORT 	      ,
GET_RELAY_PORT 	      ,
SET_LED_PORT_0 		  ,
GET_LED_PORT_0 		  ,
SET_LED_PORT_1 		  ,
GET_LED_PORT_1 		  ,
ALERT 				  ,
SET_BUTTON_LOCK 	  ,
GET_BUTTON_LOCK 	  ,

/**
 * @note Additional registers:
 * GET_BOOT_MODE = 24,
 * SET_RESET_STATUS = 25,
 * 
 * These registers are only used by the U-Boot 
 * and are therefore not listed here.
 */

NUMBER_OF_CONEGX_REGISTERS,
};

#define WRITE 1
#define READ 0
#define INPUT 1
#define OUTPUT 0

/* GPIO NUMBERS */
enum GPIO_Numbers {
IO_RELAY_1     ,
IO_RELAY_2     ,
IO_RELAY_3     ,
IO_RELAY_4     ,
IO_RESERVED_1  , // Former IO_MRES_M2    
IO_RESERVED_2  , // Former IO_MRES_M1    
IO_RESERVED_3  , // Former IO_MRES_S2    
IO_RESERVED_4  , // Former IO_MRES_S1    
IO_RESERVED_5  , // Former IO_FLT_HBUS24 
IO_RESERVED_6  , // Former IO_FLT_HBUS   
IO_RST_BUTTON  ,
IO_TST_BUTTON  ,
IO_PFI_1 	   ,
IO_PFI_2 	   ,
IO_PFI_3 	   ,
IO_PFI_4 	   ,

NUMBER_OF_CONEGX_GPIOS,
};

/* LED NUMBERS */
enum LED_Numbers {
IO_LED_1      ,
IO_LED_2      ,
IO_LED_3      ,
IO_LED_4      ,
IO_LED_5      ,
IO_LED_6      ,
IO_RGBLED_1_1 ,
IO_RGBLED_1_2 ,
IO_RGBLED_1_3 ,

NUMBER_OF_CONEGX_LEDS,
};


/* ------------------------------IRQ------------------------------ */

/* IRQ EDGES */
#define FALLING_EDGE 0
#define RISING_EDGE 1

/* IRQ NUMBERS */
enum IRQ_Numbers {
NO_INTERRUPT						, 
POTENTIAL_FREE_INPUT_1_RISING_EDGE	,
POTENTIAL_FREE_INPUT_1_FALLING_EDGE	,
POTENTIAL_FREE_INPUT_2_RISING_EDGE	,
POTENTIAL_FREE_INPUT_2_FALLING_EDGE	,
POTENTIAL_FREE_INPUT_3_RISING_EDGE	,
POTENTIAL_FREE_INPUT_3_FALLING_EDGE	,
POTENTIAL_FREE_INPUT_4_RISING_EDGE	,
POTENTIAL_FREE_INPUT_4_FALLING_EDGE	,
I2C_EXPANDER_HARDWARE_MALFUNCTION   ,
WATCHDOG_RESET                      ,
RESET_BUTTON_PRESSED				,
RESET_BUTTON_RELEASED				,
TEST_BUTTON_PRESSED					,
TEST_BUTTON_RELEASED				,
POWER_ON_RESET						,

NUMBER_OF_CONEGX_IRQS               ,
};
/* LED */
#define NR_OF_LEDS 9
#define LED_FULL 255


/**
 * @brief conegx_gpio_irq_map [GpioNr, Edge]
 * @description: maps IRQ Numbers to GPio Pins and Edges 
 */
const int conegx_gpio_irq_map[8][2] = {
	{IO_PFI_1, RISING_EDGE} ,	//	PFI 1
	{IO_PFI_1, FALLING_EDGE},   //	PFI 1
	{IO_PFI_2, RISING_EDGE} ,	//	PFI 2
	{IO_PFI_2, FALLING_EDGE},   //	PFI 2
	{IO_PFI_3, RISING_EDGE} ,	//	PFI 3
	{IO_PFI_3, FALLING_EDGE},   //	PFI 3
	{IO_PFI_4, RISING_EDGE} ,	//	PFI 4
	{IO_PFI_4, FALLING_EDGE},   //	PFI 4
};

/**
 * @brief CONEGX REGISTER access 
 * READ= read acces only
 * WRITE=  write acces only
 */
const bool conegx_reg_access[NUMBER_OF_CONEGX_REGISTERS] = {
	READ , // Get Device Description	
	READ , // Get FW Version Major
	READ , // Get FW Version Minor 1
	READ , // Get FW Version Minor 2
	WRITE, // Set OS READy
	READ , // Get Input Port
	WRITE, // Set Relay Port
	READ , // Get Relay Port
	WRITE, // Set LED Port 0
	READ , // Get LED Port 0
	WRITE, // Set LED Port 1
	READ , // Get LED Port 1
	READ , // Alert
	WRITE, // Set Button Lock
	READ , // Get Button Lock	
};

const char *const conegx_gpio_names[NUMBER_OF_CONEGX_GPIOS] = {
	"S_1"		,
	"S_2"		,
	"W_3"		,
	"W_4"		,
	"Reserved_1", // Former "MRES_M2"
	"Reserved_2", // Former "MRES_M1"
	"Reserved_3", // Former "MRES_S2"
	"Reserved_4", // Former "MRES_S1"
	"Reserved_5", // Former "FLT_HBUS24"
	"Reserved_6", // Former "FLT_HBUS"
	"RST_Butt"  ,
	"TST_Butt"  ,
	"SwitchIN_1",
	"SwitchIN_2",
	"SwitchIN_3",
	"SwitchIN_4",
};

const char *const conegx_led_names[NUMBER_OF_CONEGX_LEDS] = {
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

const int conegx_directions[NUMBER_OF_CONEGX_GPIOS] = {
	OUTPUT,	// IO_RELAY_1   
	OUTPUT,	// IO_RELAY_2   
	OUTPUT,	// IO_RELAY_3   
	OUTPUT,	// IO_RELAY_4   
	-1    ,	// Reserved_1
	-1    ,	// Reserved_2
	-1    ,	// Reserved_3
	-1    ,	// Reserved_4
	-1    ,	// Reserved_5
	-1    ,	// Reserved_6
	INPUT ,	// IO_RST_BUTTON
	INPUT ,	// IO_TST_BUTTON
	INPUT ,	// IO_PFI_1
	INPUT ,	// IO_PFI_2
	INPUT ,	// IO_PFI_3
	INPUT ,	// IO_PFI_4
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
	__u8 SetLedPort0Buffer;
	__u8 SetLedPort1Buffer;

	/* Device Status Info */
	uint LastInterruptNr;
	char FwVersion[FW_VERSION_STRING_SIZE];
	int RelayDefaultSetting;
	int TstButtonLock;
	int RstButtonLock;
	int IRQDeviceFileEnabled;
};


#endif // CONEGX_DRIVER
