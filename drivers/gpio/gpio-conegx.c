/**
 * @file gpio-conegx.c
 * @author A. Pietsch (a.pietsch@consolinno.de)
 * @brief Driver for Consolinno Conegx Module
 * @version 1.1.0
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

/*This enables some Prints and IRQ Testing for powerfail and voltagerange.
when enabled triggering MRES irqs will simulate IRQS for powerfail and 
voltagerange
*/
//#define CONEGX_TESTING

#include "gpio-conegx.h"

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/kdev_t.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/proc_fs.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#define ldev_to_led(c) container_of(c, struct conegx_led, ldev)

/* Global Variables ---------------------------------------------------------*/
static struct conegx *Conegx;

/* Device File */
static dev_t ConDevNr = 0;
static struct class *ConDevClass;
static struct device *ConDevice;
static struct cdev *ConDriverObject;

/* IRQ */
static wait_queue_head_t IrSleepingQeue;
static int InterruptArrived = 0;

/* Proc FS */
static struct proc_dir_entry *ProcfsParent;

/* Function Prototypes */
static int reset_MSP430(void);

/*---------------GPIO Functions---------------*/
static int conegx_get_direction(struct gpio_chip *chip, unsigned offset);
static int conegx_get_gpio(struct gpio_chip *chip, unsigned offset);
static int conegx_set_gpio(unsigned offset, int value);
static void set_gpio(struct gpio_chip *chip, unsigned offset, int value);
static int conegx_direction_input(struct gpio_chip *chip, unsigned offset);
static int conegx_direction_output(struct gpio_chip *chip, unsigned offset,
                                   int val);
/*---------------PROCFS Functions---------------*/
static ssize_t read_proc_fwversion(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset);

static ssize_t write_proc_tstbuttonlock(
    struct file *filp, 
    const char *buff,
    size_t len, 
    loff_t *off);

static ssize_t read_proc_tstbuttonlock(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset);

static ssize_t write_proc_rstbuttonlock(
    struct file *filp, 
    const char *buff,
    size_t len, 
    loff_t *off);

static ssize_t read_proc_rstbuttonlock(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset);

static ssize_t read_proc_resetmsp(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset);

/**
 * @brief Struct for Register Map Configuration
 * 
 */
const struct regmap_config ConegxRegmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = GET_BUTTON_LOCK << 1,
    //.reg_defaults = conegx_defaults,
    .num_reg_defaults = ARRAY_SIZE(conegx_reg_access)};
EXPORT_SYMBOL_GPL(ConegxRegmap);

/*---------------GPIO---------------------------------------------------------*/

/**
 * @brief Get Direction of the GPIOs
 * 
 * @param chip Gpio Chip
 * @param offset Gpio Number 
 * @return int  succesfull returns 0 , failure -1
 */
static int conegx_get_direction(struct gpio_chip *chip, unsigned offset) {
    /* RELAY PORT */
    if(IO_RELAY_1 <= offset && offset <= IO_RELAY_4) 
    {
        return 0;  //OUTPUT
    }
    /* HBUS */
    else if(IO_MRES_M2 <= offset && offset <= IO_MRES_S1) 
    {
        /* TODO: READ FROM MSP */
        return 1;  //INPUT
    }
    /* INPUT PORT */
    else if(IO_FLT_HBUS24 <= offset && offset <= IO_PFI_4) 
    {
        return 1;  //INPUT
    }

    return 0;
}

/**
 * @brief Get Status of the GPIOs
 * 
 * @param chip Gpio Chip
 * @param offset Gpio Number 
 * @return int  succesfull returns 0 , failure -1
 */
static int conegx_get_gpio(struct gpio_chip *chip, unsigned offset) 
{
    uint Buffer;
    int Ret = 0;
    int RegisterAdress = 0;

    Buffer = 0;

    pr_info("conegx: getting gpio %d %s\n", offset, conegx_gpio_names[offset]);

    /* Shift offset/pin to conegx numbers */

    /* read GET RELAY PORT */
    if(IO_RELAY_1 <= offset && offset <= IO_RELAY_4) 
    {
        RegisterAdress = GET_RELAY_PORT;
    }
    /* read GET INPUT PINS */
    else if(IO_FLT_HBUS24 <= offset && offset <= IO_PFI_4) 
    {
        RegisterAdress = GET_INPUT_PORT;
        offset -= IO_FLT_HBUS24;
    } 
    else 
    {
        return -1;
    }

    mutex_lock(&Conegx->lock);
    Ret = regmap_read(Conegx->regmap, RegisterAdress, &Buffer);
    pr_info("conegx: Reading Register 0x%x: 0x%x\n", RegisterAdress, Buffer);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error reading conegx gpio %d\n", offset);

        reset_MSP430();

        mutex_unlock(&Conegx->lock);
        return Ret;
    }

    mutex_unlock(&Conegx->lock);
    return !!(Buffer & BIT(offset));
}

/**
 * @brief Get GPio Status
 * 
 * @param offset Gpio Number	
 * @param value 1 for Active/HIGH, 0 for Inactive/LOW
 * @return int  succesfull returns 0 , failure -1
 */
static int conegx_set_gpio(unsigned offset, int value) 
{
    uint Buffer;
    int Ret, RegisterAdress = 0;
    int InternalRegisterOffset;
    __u8 *RegisterBuffer = NULL;

    pr_info("conegx: setting gpio %d %s to %d\n",
            offset, conegx_gpio_names[offset], value);

    /* Shift offset/pin to conegx numbers */

    /* Get InternalRegister and InternalRegister Offset */
    if(IO_RELAY_1 <= offset && offset <= IO_RELAY_4) 
    {
        RegisterAdress = SET_RELAY_PORT;
        RegisterBuffer = &Conegx->SetRelayBuffer;
        Buffer = Conegx->SetRelayBuffer;
        InternalRegisterOffset = offset - IO_RELAY_1;
    } 

    /* Modify Bits in Buffer */
    if(value)
    {
        Buffer |= BIT(InternalRegisterOffset);
    }
    else
    {
        Buffer &= ~BIT(InternalRegisterOffset);
    }
    /* Write buffer to Register */
    mutex_lock(&Conegx->lock);
    Ret = regmap_write(Conegx->regmap, RegisterAdress, Buffer);
    pr_info("conegx: Writing Register 0x%x: 0x%x\n", RegisterAdress, Buffer);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error writing to Register 0x%x\n",
               RegisterAdress);

        reset_MSP430();

        mutex_unlock(&Conegx->lock);
        return -1;  
    } 
    else 
    {
        /* Update RegisterBuffer */
        *RegisterBuffer = Buffer;
    }

    mutex_unlock(&Conegx->lock);
    return 0;
}

/**
 * @brief Set the gpio Value
 * 
 * @param chip Gpio Chip
 * @param offset Gpio Number
 * @param value High or Low Value
 */
static void set_gpio(struct gpio_chip *chip, unsigned offset, int value) 
{
    /* call Conegx Gpio Set Function */
    conegx_set_gpio(offset, value);
}

/**
 * @brief Set Direction to INPUT 
 * 
 * @param chip Gpio Chip
 * @param offset Gpio Number
 * @return int  succesfull returns 0 , failure -1
 */
static int conegx_direction_input(struct gpio_chip *chip, unsigned offset) 
{
    int Ret;

    pr_info("conegx: setting direction INPUT for gpio %d %s\n",
            offset, conegx_gpio_names[offset]);

    /* Return error for Relais Outputs */
    if(IO_RELAY_1 <= offset && offset <= IO_RELAY_4) 
    {
        return -1;
    }
    
    //return conegx_get_gpio(chip,offset);
    return 0;
}

/**
 * @brief Set Gpio Direction to Output
 * 
 * @param chip Gpio Chip
 * @param offset Gpio Number
 * @param val Inital Gpio Status after setting: High or Low 
 * @return int  succesfull returns 0 , failure -1
 */
static int conegx_direction_output(
    struct gpio_chip *chip,
    unsigned offset, 
    int val) 
{
    int Ret;
    int InternalRegisterOffset;

    pr_info("conegx: setting direction OUTPUT for gpio %d %s\n",
            offset, conegx_gpio_names[offset]);

    if(IO_FLT_HBUS24 <= offset && offset <= IO_PFI_4) 
    {
        return -1;
    }
    // set actual gpio values
    return conegx_set_gpio(offset, val);
}
/*---------------FS-----------------------------------------------------------*/

/**
 * @brief Read Function of the device File /dev/conegx
 */
static ssize_t con_devfile_read(
    struct file *instanz, 
    char __user *user,
    size_t count, 
    loff_t *offset) 
{
    char IRQNumberChar[4];
    int BytesRead;
    int BytesToRead = sizeof(IRQNumberChar) - *offset;

    memset(IRQNumberChar, 0, sizeof(IRQNumberChar));

    // If we are at the end of the file, STOP READING!
    if(BytesToRead == 0) 
    {
        return BytesToRead;
    }

    /* Wait for Change */
    pr_info("conegx: Someone is now listening to DevFile for IRQ numbers\n");
    Conegx->IRQDeviceFileEnabled = 1;
    InterruptArrived = 0;
    wait_event_interruptible(IrSleepingQeue, InterruptArrived);

    /* GetCharversion */
    sprintf(IRQNumberChar, "%-2d\n", Conegx->LastInterruptNr);

    // Get bytes read by subtracting return of copy_to_user (returns unread bytes)

    BytesRead = BytesToRead - copy_to_user(user, IRQNumberChar + *offset,
                                           BytesToRead);

    // Set offset so that we can eventually reach the end of the file
    *offset += BytesRead;
    Conegx->IRQDeviceFileEnabled = 0;
    return BytesRead;
}

/**
 * @brief File Operation Struct for /dev/conegx
 */
static struct file_operations fops_devfile = {
    .owner = THIS_MODULE,
    .read = con_devfile_read,
};

/*---------------PROCFS-------------------------------------------------------*/
/**
 * @brief File Operation Struct for /proc/conegx/tstbuttonlock
 */
static struct file_operations proc_fops_tstbuttonlock = {

    .read = read_proc_tstbuttonlock,
    .write = write_proc_tstbuttonlock,

};

/**
 * @brief File Operation Struct for /proc/conegx/rstbuttonlock
 */
static struct file_operations proc_fops_rstbuttonlock = {

    .read = read_proc_rstbuttonlock,
    .write = write_proc_rstbuttonlock,

};

/**
 * @brief File Operation Struct for /proc/conegx/fwversion
 */
static struct file_operations proc_fops_fwversion = {

    .read = read_proc_fwversion,

};

/**
 * @brief File Operation Struct for /proc/conegx/resetmsp
 */
static struct file_operations proc_fops_resetmsp = {

    .read = read_proc_resetmsp,

};

/**
 * @brief Read Function  for /proc/conegx/fwversion
 */
static ssize_t read_proc_fwversion(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset) 
{
    int BytesRead;
    int BytesToRead = sizeof(Conegx->FwVersion) - *offset;

    // If we are at the end of the file, STOP READING!
    if(BytesToRead == 0) 
    {
        return BytesToRead;
    }

    // Get bytes read by subtracting return of copy_to_user
    BytesRead = BytesToRead - copy_to_user(buffer, Conegx->FwVersion + *offset,
                                           BytesToRead);

    printk("conegx: Reading %d bytes Fw Version: %s\n",
           BytesRead, Conegx->FwVersion);

    // Set offset so that we can eventually reach the end of the file
    *offset += BytesRead;
    return BytesRead;
}

static ssize_t read_proc_resetmsp(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset)
{
    reset_MSP430();

    return 0;
}

/**
 * @brief Read Function  for /proc/conegx/testbuttonlock
 */
static ssize_t read_proc_tstbuttonlock(
    struct file *filp,
    char __user *buffer,
    size_t length,
    loff_t *offset) 
{
    char TstButtonLockChar[2];
    int BytesRead;
    int BytesToRead = 2 - *offset;

    TstButtonLockChar[0] = (char)(Conegx->TstButtonLock + '0');
    TstButtonLockChar[1] = '\n';

    // If we are at the end of the file, STOP READING!
    if(BytesToRead == 0) 
    {
        return BytesToRead;
    }

    // Get bytes read by subtracting return of copy_to_user
    BytesRead = BytesToRead - copy_to_user(buffer, TstButtonLockChar + *offset,
                                           BytesToRead);

    printk("conegx: Reading %d bytes TstButon Range: %c\n",
           BytesRead, TstButtonLockChar[0]);

    // Set offset so that we can eventually reach the end of the file
    *offset += BytesRead;
    return BytesRead;
}

/**
 * @brief Write Function  for /proc/conegx/testbuttonlock
 */
static ssize_t write_proc_tstbuttonlock(
    struct file *filp,
    const char *buff,
    size_t len,
    loff_t *off) 
{
    int Ret;
    unsigned long long TstButtonLockBuffer;

    Ret = kstrtoull_from_user(buff, len, 10, &TstButtonLockBuffer);
    if(Ret) 
    {
        /* Negative error code. */
        pr_info("conegx: Error converting ButtonLock RetVal = %d\n", Ret);
        return Ret;
    } 
    else 
    {
        /* Check if Value is in Range */
        if(TstButtonLockBuffer == 1 || TstButtonLockBuffer == 0) 
        {
            /* Set Button Lock for Tst button */
            Conegx->TstButtonLock = TstButtonLockBuffer;
        } 
        else 
        {
            return -1;
        }

        printk(KERN_INFO "conegx: Setting ButtonLock to: %d \n",
               (Conegx->TstButtonLock | (Conegx->RstButtonLock << 4)));
        *off = len;

        /* Write Setting to Conegx */
        Ret = regmap_write(Conegx->regmap, SET_BUTTON_LOCK,
                           (Conegx->TstButtonLock |
                            (Conegx->RstButtonLock << 4)));
        if(Ret) 
        {
            printk(KERN_ERR
                   "conegx: Error writing to Register \
							SET_BUTTON_LOCK\n");

            reset_MSP430();

            return -1;
        }
    }

    return len;
}

/**
 * @brief Read Function  for /proc/conegx/rstbuttonlock
 */
static ssize_t read_proc_rstbuttonlock(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset) 
{
    char RstButtonLockChar[2];
    int BytesRead;
    int BytesToRead = 2 - *offset;

    RstButtonLockChar[0] = (char)(Conegx->RstButtonLock + '0');
    RstButtonLockChar[1] = '\n';

    // If we are at the end of the file, STOP READING!
    if(BytesToRead == 0) 
    {
        return BytesToRead;
    }

    // Get bytes read by subtracting return of copy_to_user
    BytesRead = BytesToRead - copy_to_user(buffer,
                                           RstButtonLockChar + *offset,
                                           BytesToRead);
    printk("conegx: Reading %d bytes TstButon Range: %c\n",
           BytesRead, RstButtonLockChar[0]);

    // Set offset so that we can eventually reach the end of the file
    *offset += BytesRead;
    return BytesRead;
}

/**
 * @brief Write Function  for /proc/conegx/rstbuttonlock
 */
static ssize_t write_proc_rstbuttonlock(
    struct file *filp,
    const char *buff,
    size_t len, 
    loff_t *off) 
{
    int Ret;
    unsigned long long RstButtonLockBuffer;

    Ret = kstrtoull_from_user(buff, len, 10, &RstButtonLockBuffer);
    if(Ret) 
    {
        /* Negative error code. */
        pr_info("conegx: Error converting ButtonLock RetVal = %d\n", Ret);
        return Ret;
    } 
    else 
    {
        /* Check if Value is in Range */
        if(RstButtonLockBuffer == 1 || RstButtonLockBuffer == 0) 
        {
            /* Set Button Lock for Tst button */
            pr_info("conegx: ResetButtonLockbuffer = %lld\n",
                    RstButtonLockBuffer);

            Conegx->RstButtonLock = RstButtonLockBuffer;
        } 
        else 
        {
            return -1;
        }

        printk(KERN_INFO "conegx: Setting ButtonLock to: %d \n",
               (Conegx->TstButtonLock | (Conegx->RstButtonLock << 4)));

        *off = len;

        /* Write Setting to Conegx */
        Ret = regmap_write(Conegx->regmap, SET_BUTTON_LOCK,
                           (Conegx->TstButtonLock |
                            (Conegx->RstButtonLock << 4)));
        if(Ret) 
        {
            printk(KERN_ERR
                   "conegx: Error writing to Register \
			SET_BUTTON_LOCK\n");

            reset_MSP430();

            return -1;
        }
    }

    return len;
}

/*---------------IRQ----------------------------------------------------------*/
/**
 * @brief IRQ Handler
 */
static irqreturn_t conegx_irq(int irq, void *data) 
{
    int Ret;
    uint IrqNumber;
    int ChildIRQ;
    int GpioNumber;
    int Edge;

    mutex_lock(&Conegx->lock);

    /* Read Alert Register */
    Ret = regmap_read(Conegx->regmap, ALERT, &IrqNumber);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error reading ALERT REGISTER\n");
        mutex_unlock(&Conegx->lock);
        return Ret;
    }

    mutex_unlock(&Conegx->lock);
    Conegx->LastInterruptNr = IrqNumber;

    pr_info("conegx: IRQ detected. Interrupt Nr.: %d\n", IrqNumber);

    /* GPIO INTERRUPTS -------------------*/
    if(IrqNumber >= POTENTIAL_FREE_INPUT_1_RISING_EDGE &&
        IrqNumber <= MRES_S2_FALLING_EDGE) 
    {
        /* Get Gpio Number and Edge from IRQ Number */
        GpioNumber = conegx_gpio_irq_map[IrqNumber -
                                         POTENTIAL_FREE_INPUT_1_RISING_EDGE][0];

        Edge = conegx_gpio_irq_map[IrqNumber -
                                   POTENTIAL_FREE_INPUT_1_RISING_EDGE][1];

        pr_info("conegx: Interrupt on GPIONR: %d",
                GpioNumber);

        if(Edge) 
        {
            pr_info("conegx: Rising Edge on %s\n",
                    conegx_gpio_names[GpioNumber]);
        } 
        else 
        {
            pr_info("conegx: Falling Edge on %s\n",
                    conegx_gpio_names[GpioNumber]);
        }

        /* This wakeups are just for testing */
        #ifdef CONEGX_TESTING
        VoltageInterruptArrived += 1;
        PowerFailInterruptArrived += 1;      
        #endif // DEBUG
        
        //#ifdef GPIOLIB_IRQCHIP
        /* Trigger nested IRQ for GPIOS */
        ChildIRQ = irq_find_mapping(Conegx->chip.irq.domain, GpioNumber);
        pr_info("conegx: handling childirq %d\n", ChildIRQ);
        handle_nested_irq(ChildIRQ);

        //#endif
    }

    /* Check if any IRQ is enabled and wake up Sleeping Queue */
    if(Conegx->IRQDeviceFileEnabled || Conegx->IRQVoltageRangeEnabled ||
        Conegx->IRQPowerFailEnabled) 
    {
        InterruptArrived += 1;
        wake_up(&IrSleepingQeue);
    }
    return IRQ_HANDLED;
}
/* LED -----------------------------------------------------------------------*/

/**
 * @brief Led Set Brightness Function
 */
static int conegxled_set_brightness(
    struct led_classdev *led_cdev,
    enum led_brightness value) 
{
    uint Buffer;
    int Ret, RegisterAdress = 0;
    int InternalRegisterOffset;
    __u8 *RegisterBuffer = NULL;
    struct conegx_led *led = ldev_to_led(led_cdev);

    if(IO_LED_1 <= led->led_no && led->led_no <= IO_LED_6) 
    {
        RegisterAdress = SET_LED_PORT_0;
        RegisterBuffer = &Conegx->SetLedPort0Buffer;
        Buffer = Conegx->SetLedPort0Buffer;
        InternalRegisterOffset = led->led_no;
    } 
    else if(IO_RGBLED_1_1 <= led->led_no && led->led_no <= IO_RGBLED_1_3) 
    {
        RegisterAdress = SET_LED_PORT_1;
        RegisterBuffer = &Conegx->SetLedPort1Buffer;
        Buffer = Conegx->SetLedPort1Buffer;
        InternalRegisterOffset = led->led_no - IO_RGBLED_1_1;
    }

    /* Modify Bits in Buffer */
    if(value) 
    {
        pr_info("conegx: Turn ON LED Number: %d %s\n", led->led_no, led->name);
        Buffer |= BIT(InternalRegisterOffset);
    } 
    else
    {
        Buffer &= ~BIT(InternalRegisterOffset);
        pr_info("conegx: Turn OFF LED Number: %d %s\n", led->led_no, led->name);
    }
    mutex_lock(&Conegx->lock);
    /* Write buffer to Register */
    Ret = regmap_write(Conegx->regmap, RegisterAdress, Buffer);
    pr_info("conegx: Writing Register 0x%x: 0x%x\n", RegisterAdress, Buffer);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error writing to Register 0x%x\n",
               RegisterAdress);

        reset_MSP430();

        mutex_unlock(&Conegx->lock);
        return -1;
    } 
    else 
    {
        /* Update RegisterBuffer */
        *RegisterBuffer = Buffer;
    }
    mutex_unlock(&Conegx->lock);
    return 0;
}

/**
 * @brief Remove function for LEDS
 */
static int unregister_leds(int NrOfLeds) 
{
    int i;
    /* unregister already registered leds */

    for (i = 0; i < NrOfLeds; i++) 
    {
        led_classdev_unregister(&Conegx->leds[i].ldev);
    }
    return 0;
}

/**
 * @brief Setup Function for LEDS
 */
static int setup_leds(struct i2c_client *client) 
{
    unsigned int i;
    int Err;
    pr_info("conegx: Setting up Leds\n");
    for (i = 0; i < NR_OF_LEDS; i++) 
    {
        struct conegx_led *Led = &Conegx->leds[i];
        Led->led_no = i;
        Led->name = conegx_led_names[i];
        Led->ldev.brightness_set_blocking = conegxled_set_brightness;
        Led->ldev.max_brightness = LED_FULL;
        Led->ldev.name = conegx_led_names[i];
        //Led->ldev.default_trigger = NULL;
        Err = led_classdev_register(&client->dev, &Led->ldev);
        if(Err < 0) 
        {
            dev_err(&client->dev, "couldn't register LED %s\n",
                    Led->ldev.name);
            unregister_leds(i);
            return -1;
        }
        mutex_lock(&Led->ldev.led_access);
		led_sysfs_enable(&Led->ldev);
		mutex_unlock(&Led->ldev.led_access);
    }

    return 0;
}

/**
 * @brief Function that Mirrors all Conegx Registers to the driver at startup
 */
static int conegx_getRegister(void) 
{
    int Ret;
    int Val;
    int FwVersionMaj;
    int FwVersionMin;
    int FwVersionPatch;

    pr_info("conegx: Collecting Device Infos:\n");
    /* Reading GPIO and LED States into buffers Register to identify chip */

    mutex_lock(&Conegx->lock);

    Ret = regmap_read(Conegx->regmap, GET_RELAY_PORT, &Val);
    if(Ret < 0) 
    {
        printk(KERN_ERR "conegx: can't read GET_RELAY_PORT Register\n");
    } 
    else 
    {
        Conegx->SetRelayBuffer = (char)(Val & 0xFF);
        pr_info("conegx: GET_RELAY_PORT: 0x%x\n", Val);
    }

    Ret = regmap_read(Conegx->regmap, GET_LED_PORT_0, &Val);
    if(Ret < 0) 
    {
        printk(KERN_ERR "conegx: can't read GET_LED_PORT_0 Register\n");
    } 
    else 
    {
        Conegx->SetLedPort0Buffer = (char)(Val & 0xFF);
        pr_info("conegx: GET_LED_PORT_0: 0x%x\n", Val);
    }

    Ret = regmap_read(Conegx->regmap, GET_LED_PORT_1, &Val);
    if(Ret < 0) 
    {
        printk(KERN_ERR "conegx: can't read GET_LED_PORT_1 Register\n");
    } 
    else 
    {
        Conegx->SetLedPort1Buffer = (char)(Val & 0xFF);
        pr_info("conegx: GET_LED_PORT_1: 0x%x\n", Val);
    }

    /* Get Fw Version */
    Ret = regmap_read(Conegx->regmap, FW_VERSION_MAJOR, &FwVersionMaj);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read FW_VERSION_MAJOR Register\n");
        mutex_unlock(&Conegx->lock);
        return Ret;
    }
    Ret = regmap_read(Conegx->regmap, FW_VERSION_MINOR_1, &FwVersionMin);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read FW_VERSION_MINOR_1 Register\n");
        mutex_unlock(&Conegx->lock);
        return Ret;
    }
    Ret = regmap_read(Conegx->regmap, FW_VERSION_MINOR_2, &FwVersionPatch);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read FW_VERSION_MINOR_2 Register\n");
        mutex_unlock(&Conegx->lock);
        return Ret;
    }

    /* Create Character Version of String */
    sprintf(Conegx->FwVersion, "%d.%d.%d\n", FwVersionMaj, FwVersionMin,
            FwVersionPatch);

    pr_info("conegx: FirmwareVersion: %s", Conegx->FwVersion);

    /* Get Button Lock Setting */
    Ret = regmap_read(Conegx->regmap, GET_BUTTON_LOCK, &Val);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read GET_BUTTON_LOCK Register\n");
        mutex_unlock(&Conegx->lock);
        return Ret;
    }
    Conegx->TstButtonLock = (Val & 0x1);
    Conegx->RstButtonLock = (Val & 0x10) >> 0x4;
    pr_info("conegx: RstButtonLock: %d", Conegx->RstButtonLock);
    pr_info("conegx: TstButtonLock: %d", Conegx->TstButtonLock);

    mutex_unlock(&Conegx->lock);
    return 0;
}

/**
 * @brief Probe Function for the Conegx Device. Initializes the Module
 */
static int conegx_probe(struct i2c_client *client) {
    int Ret;
    int Err;
    unsigned int Val;
    unsigned long IrqFlags = IRQF_ONESHOT | IRQF_TRIGGER_RISING;

    #ifdef CONEGX_TESTING
    printk("conegx: Loaded in Testmode");
    #endif // DEBUG
    pr_info("conegx: Driver Version: %s",DRIVER_VERSION);
    pr_info("conegx: runnning probe for %s @ 0x%x", client->name, client->addr);

    Conegx = devm_kzalloc(&client->dev, sizeof(*Conegx), GFP_KERNEL);
    if(!Conegx)
    {
        return -ENOMEM;
    }

    Conegx->dev = &client->dev;
    Conegx->addr = client->addr;
    Conegx->irq = client->irq;

    /* Initialize Regmap */
    Conegx->regmap = devm_regmap_init_i2c(client, &ConegxRegmap);

    Conegx->irq_chip.name = dev_name(Conegx->dev);

    /* GPIO -----------------------------------------------------------------*/
    Conegx->chip.label = client->name;
    Conegx->chip.parent = &client->dev;
    Conegx->chip.owner = THIS_MODULE;
    Conegx->chip.get_direction = conegx_get_direction;
    Conegx->chip.get = conegx_get_gpio;
    Conegx->chip.set = set_gpio;
    Conegx->chip.direction_input = conegx_direction_input;
    Conegx->chip.direction_output = conegx_direction_output;
    Conegx->chip.base = -1;
    Conegx->chip.names = conegx_gpio_names;
    Conegx->chip.ngpio = 16;
    Conegx->chip.can_sleep = true;

    Ret = devm_gpiochip_add_data(Conegx->dev, &Conegx->chip, Conegx);
    if(Ret < 0) 
    {
        printk(KERN_ERR "conegx: can't add GPIO chip\n");
    }

    mutex_init(&Conegx->lock);

    /* add Data to I2c and GPIO */
    i2c_set_clientdata(client, Conegx);

    /* IRQ -----------------------------------------------------------------*/

    Ret = devm_request_threaded_irq(Conegx->chip.parent, Conegx->irq, NULL,
                                    conegx_irq,
                                    IrqFlags, "conegxirq", Conegx);

    if(Ret != 0) 
    {
        dev_err(Conegx->dev, "conegx: unable to request IRQ#%d: %d\n",
                Conegx->irq, Ret);
        return Ret;
    } 
    else 
    {
        pr_info("conegx: registered IRQ # %d\n",
                Conegx->irq);
    }

    /* Setting up GPio IRQ */
    Err = gpiochip_irqchip_add_nested(&Conegx->chip,
                                      &Conegx->irq_chip,
                                      0,
                                      handle_edge_irq,
                                      IRQ_TYPE_NONE);

    Conegx->chip.irq.threaded = true;

    if(Err) 
    {
        dev_err(Conegx->dev,
                "could not connect irqchip to gpiochip: %d\n", Err);
        return Err;
    }

    gpiochip_set_nested_irqchip(&Conegx->chip,
                                &Conegx->irq_chip,
                                Conegx->irq);

    /* Reading FW_VERSION Register to identify chip*/
    mutex_lock(&Conegx->lock);
    Ret = regmap_read(Conegx->regmap, DEVICE_DESCRIPTION, &Val);
    if(Ret < 0) 
    {
        printk(KERN_ERR "conegx: can't read DEVICE_DESCRIPTION Register\n");
    }
    mutex_unlock(&Conegx->lock);
    if(Val != 0x94) 
    {
        pr_info("conegx: DEVICE_DESCRIPTION wrong (!0x94): 0x%x\n", Val);
        return 1;
    }
    else
    {
        pr_info("conegx: valid DEVICE_DESCRIPTION (0x94)!\n");
    }

    Ret = conegx_getRegister();

    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error getting Device Data\n");
    }

    /* PROCFS ---------------------------------------------------------------*/
    ProcfsParent = proc_mkdir("conegx", NULL);

    if(ProcfsParent == NULL) 
    {
        pr_info("Error creating proc entry");
    }

    /*Creating Proc entry under "/proc/etx/" */
    proc_create("fwversion", 0444, ProcfsParent, &proc_fops_fwversion);
    proc_create("tstbuttonlock", 0666, ProcfsParent, &proc_fops_tstbuttonlock);
    proc_create("rstbuttonlock", 0666, ProcfsParent, &proc_fops_rstbuttonlock);
    proc_create("resetmsp", 0444, ProcfsParent, &proc_fops_resetmsp);

    /* LEDS -----------------------------------------------------------------*/
    setup_leds(client);

    init_waitqueue_head(&IrSleepingQeue);

    if(alloc_chrdev_region(&ConDevNr, 0, 1, "conegx_device") < 0)
    {
        return -EIO;
    }
    ConDriverObject = cdev_alloc(); /* Anmeldeobjekt reservieren */
    if(ConDriverObject == NULL)
    {
        goto free_device_number;
    }
    ConDriverObject->owner = THIS_MODULE;
    ConDriverObject->ops = &fops_devfile;
    if(cdev_add(ConDriverObject, ConDevNr, 1))
    {
        goto free_cdev;
    }
    ConDevClass = class_create(THIS_MODULE, "conegx_class");
    if(IS_ERR(ConDevClass)) 
    {
        pr_err("conegx_class: no udev support\n");
        goto free_cdev;
    }
    ConDevice = device_create(ConDevClass, NULL, ConDevNr,
                            NULL, "%s", "conegx");

    if(IS_ERR(ConDevice))
    {
        goto free_class;
    }
    //dev_info(ConDevice, "mod_init");

    /* Set OS Ready flag ----------------------------------------------------*/
    Ret = regmap_write(Conegx->regmap, SET_OS_READY, 0x1);
    pr_info("conegx: Device Initialzed -> Setting OS_READY flag\n");
    if (Ret) {
        printk(KERN_ERR "conegx: Error writing to SET_OS_READY\n");

        reset_MSP430();
    }

    return 0;

free_class:
    class_destroy(ConDevClass);
free_cdev:
    kobject_put(&ConDriverObject->kobj);
free_device_number:
    unregister_chrdev_region(ConDevNr, 1);
    return -EIO;
}

/**
 * @brief Remove Function called when the module is unloaded
 */
static int conegx_remove(struct i2c_client *client) 
{
    int Ret;
    pr_info("conegx: Removing...-> disabling OS_READY flag\n");
    Ret = regmap_write(Conegx->regmap, SET_OS_READY, 0x0);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error writing to SET_OS_READY\n");
        reset_MSP430();
    }
    proc_remove(ProcfsParent);
    unregister_leds(NR_OF_LEDS);
    mutex_destroy(&Conegx->lock);
    device_destroy(ConDevClass, ConDevNr);
    class_destroy(ConDevClass);

    return 0;
}

static int reset_MSP430(void)
{
    int rv;

    pr_info("Resetting MSP430...");

    rv = gpio_request(RST_PIN, "MSP430_Reset");
    if(rv < 0)
    {
        printk(KERN_ERR "conegx: Error resetting MSP");
        return -1;
    }

    rv = gpio_direction_output(RST_PIN, 1);
    if(rv < 0)
    {
        printk(KERN_ERR "conegx: Error resetting MSP");
        gpio_free(RST_PIN);
        return -1;
    }

    gpio_set_value(RST_PIN, 0);

    mdelay(200);

    gpio_set_value(RST_PIN, 1);

    gpio_free(RST_PIN);

    /* Set OS Ready */

    return 0;
}

/* I2c Divce Infos */
static const struct i2c_device_id conegx_id_table[] = {
    {"conegx", 1},
    {/* sentinel */}};
MODULE_DEVICE_TABLE(i2c, conegx_id_table);

static const struct of_device_id conegx_of_match_table[] = {
    {.compatible = "consolinno,conegx",
     .data = (void *)1},
    {/* sentinel */}};
MODULE_DEVICE_TABLE(of, conegx_of_match_table);

static struct i2c_driver conegx_driver = {
    .driver = {
        .name = "conegx",
        .of_match_table = conegx_of_match_table,
    },
    .probe_new = conegx_probe,
    .remove = conegx_remove,
    .id_table = conegx_id_table,
};
module_i2c_driver(conegx_driver);

MODULE_AUTHOR("Alexander Pietsch <a.pietsch@consolinno.de>");
MODULE_DESCRIPTION("Driver for Consolinno Conegx");
MODULE_LICENSE("GPL v2");
