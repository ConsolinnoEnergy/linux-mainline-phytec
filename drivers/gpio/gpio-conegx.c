/**
 * @file gpio-conegx.c
 * @author S. Ardaya-Lieb (s.ardayalieb@consolinno.de)
 * @brief Driver for Consolinno Conegx Module
 * @version 1.4.0
 * 
 * @copyright: Copyrigth (c) 2021 - 2025
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
static volatile int InterruptArrived = 0;

/* Proc FS */
static struct proc_dir_entry *ProcfsParent;

/* Function Prototypes */
static int reset_MSP430(void);
static int handleReset(void);

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

static ssize_t read_proc_maintenancemode(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset);

static ssize_t write_proc_maintenancemode(
    struct file *filp, 
    const char __user *buffer,
    size_t length, 
    loff_t *offset);

static ssize_t read_proc_resetmsp(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset);

static ssize_t write_proc_resetleaflet(
    struct file *filp, 
    const char __user *buffer,
    size_t length, 
    loff_t *offset);

/**
 * @brief Struct for Register Map Configuration
 * 
 */
const struct regmap_config ConegxRegmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = GET_STATUS_PORT << 1,
    //.reg_defaults = conegx_defaults,
    .num_reg_defaults = ARRAY_SIZE(conegx_reg_access)};
EXPORT_SYMBOL_GPL(ConegxRegmap);

/*---------------GPIO---------------------------------------------------------*/

/**
 * @brief Get Direction of the GPIOs
 * 
 * @param chip Gpio Chip
 * @param offset Gpio Number 
 * @return int returns 0 for output and 1 for input, and -1 in case of failure
 */
static int conegx_get_direction(struct gpio_chip *chip, unsigned offset) 
{
    int Direction;

    pr_debug("conegx: conegx_get_direction, offset: %d\n", offset);

    if((offset >= IO_RELAY_1) && (offset <= IO_PFI_4))
    {
        Direction = conegx_directions[offset];
    }
    else
    {
        Direction = -1;
    }

    return Direction;
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
    int Ret;

    pr_debug("conegx: getting gpio %d %s\n", offset, conegx_gpio_names[offset]); 

    if(IO_RELAY_1 <= offset && offset <= IO_RELAY_4) 
    {
        mutex_lock(&Conegx->lock);
        Ret = (Conegx->RelayPortBuffer & BIT(offset)) ? 1 : 0;
        mutex_unlock(&Conegx->lock);
        
        return Ret;
    }
    else if (IO_RST_BUTTON<= offset && offset <= IO_PFI_4)
    {
        mutex_lock(&Conegx->lock);
        Ret = (Conegx->InputPortBuffer & BIT(offset - IO_RST_BUTTON + 2)) ? 1 : 0;
        mutex_unlock(&Conegx->lock);

        return Ret;
    }
    else 
    {
        return -1;
    }
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
    int Ret;
    __u8 RelayPortTmp, LedPort0Tmp, RelayBit, LedBit;

    if(IO_RELAY_1 <= offset && offset <= IO_RELAY_4) 
    {   
        pr_debug("conegx: setting gpio %d %s to %d\n", offset, conegx_gpio_names[offset], value);
        
        /* If a relay is closed or opened, the Firmware 
        turns the associated LED on or off. We reflect this behaviour here. */
        switch(offset)
        {
            case IO_RELAY_1: // S_1
            {
                RelayBit = BIT_RELAY_S1;
                LedBit = BIT_LED_S1;                    
            }break;

            case IO_RELAY_2: // S_2
            {
                RelayBit = BIT_RELAY_S2;
                LedBit = BIT_LED_S2;
            }break;

            case IO_RELAY_3: // W_3
            {
                RelayBit = BIT_RELAY_W3;
                LedBit = BIT_LED_W3;
            }break;

            case IO_RELAY_4: // W_4
            {
                RelayBit = BIT_RELAY_W4;
                LedBit = BIT_LED_W4;
            }break;

            default:
            {
                /* This should be not possible. */
                return -1;
            }            
        }

        mutex_lock(&Conegx->lock);
        if(value)
        {
            RelayPortTmp = Conegx->RelayPortBuffer | RelayBit;
            LedPort0Tmp = Conegx->LedPort0Buffer | LedBit;
        }
        else
        {
            RelayPortTmp = Conegx->RelayPortBuffer & ~(RelayBit);
            LedPort0Tmp = Conegx->LedPort0Buffer & ~(LedBit);
        }

        if (RelayPortTmp != Conegx->RelayPortBuffer)
        {
            pr_debug("conegx: Writing Register SET_RELAY_PORT: 0x%x\n", RelayPortTmp);
            Ret = regmap_write(Conegx->regmap, SET_RELAY_PORT, RelayPortTmp);
            if (Ret)
            {
                printk(KERN_ERR "conegx: Error writing to Register SET_RELAY_PORT\n");

                reset_MSP430();

                mutex_unlock(&Conegx->lock);
                return Ret;
            }
            
            /* Update register buffers. */
            Conegx->RelayPortBuffer = RelayPortTmp;
            Conegx->LedPort0Buffer = LedPort0Tmp;
        }

        mutex_unlock(&Conegx->lock);
    }
    else
    {
        pr_debug("conegx: Cannot set gpio %d to %d\n", offset, value);
        return -1;
    }

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
    pr_debug("conegx: setting direction INPUT for gpio %d %s\n", offset, conegx_gpio_names[offset]);

    /* Return error for Relais Outputs */
    if(IO_RELAY_1 <= offset && offset <= IO_RELAY_4) 
    {
        return -1;
    }
    
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
    pr_debug("conegx: setting direction OUTPUT for gpio %d %s\n",
            offset, conegx_gpio_names[offset]);

    if(IO_RESERVED_1 <= offset && offset <= IO_PFI_4) 
    {
        pr_debug("conegx: invalid offset %d", offset);

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

    mutex_lock(&Conegx->lock);
    Conegx->IRQDeviceFileEnabled = 1;
    InterruptArrived = 0;
    mutex_unlock(&Conegx->lock);

    /* Wait for Change */
    pr_debug("conegx: Someone is now listening to DevFile for IRQ numbers\n");    

    /**
     * @brief sleep until a condition gets true
     * 
     * The process is put to sleep (TASK_INTERRUPTIBLE) until the condition evaluates to true
     * or a signal is received.
     * 
     * The condition is checked each time the waitqueue wq is woken up.
     * 
     * @param wq the waitqueue to wait on
     * @param condition a C expression for the event to wait for
     * @return -ERESTARTSYS if it was interrupted by a signal 
     * and 0 if condition evaluated to true.
     */
    wait_event_interruptible(
        IrSleepingQeue,    // wq
        InterruptArrived); // condition

    mutex_lock(&Conegx->lock);
    /* GetCharversion */
    sprintf(IRQNumberChar, "%-2d\n", Conegx->LastInterruptNr);
    Conegx->IRQDeviceFileEnabled = 0;
    mutex_unlock(&Conegx->lock);  

    /* Get bytes read by subtracting return of copy_to_user (returns unread bytes) */
    BytesRead = BytesToRead - copy_to_user(user, IRQNumberChar + *offset, BytesToRead);
    /* Set offset so that we can eventually reach the end of the file */
    *offset += BytesRead;

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
 * @brief File Operation Struct for /proc/conegx/maintenance
 */
static struct file_operations proc_fops_maintenance = {

    .read = read_proc_maintenancemode,
    .write = write_proc_maintenancemode,

};

/**
 * @brief File Operation Struct for /proc/conegx/resetleaflet
 */
static struct file_operations proc_fops_resetleaflet = {
    .write = write_proc_resetleaflet,
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
    BytesRead = BytesToRead - copy_to_user(buffer, Conegx->FwVersion + *offset, BytesToRead);

    printk("conegx: Reading %d bytes Fw Version: %s\n", BytesRead, Conegx->FwVersion);

    // Set offset so that we can eventually reach the end of the file
    *offset += BytesRead;
    return BytesRead;
}

static ssize_t read_proc_maintenancemode(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset)
{
    int Ret;
    char MaintenanceModeChar[2];
    int BytesRead;
    int BytesToRead = 2 - *offset;
    int MaintenanceMode;
    unsigned int Val;    

    mutex_lock(&Conegx->lock);
    pr_debug("conegx: Reading Register 0x%x\n", GET_STATUS_PORT);
    Ret = regmap_read(Conegx->regmap, GET_STATUS_PORT, &Val);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error reading GET_STATUS_PORT Register\n");
        reset_MSP430();
        mutex_unlock(&Conegx->lock);
        return Ret;
    }

    Conegx->StatusPortBuffer = (__u8)(Val & 0xFF);
    MaintenanceMode = (Conegx->StatusPortBuffer & BIT_MAINTENANCE) ? 1 : 0;
    mutex_unlock(&Conegx->lock);

    MaintenanceModeChar[0] = (char)(MaintenanceMode + '0');
    MaintenanceModeChar[1] = '\n'; 

    /* If we are at the end of the file, STOP READING! */
    if(BytesToRead == 0) 
    {
        return BytesToRead;
    }

    BytesRead = BytesToRead - copy_to_user(
        buffer,
        MaintenanceModeChar + *offset,
        BytesToRead);

    printk("conegx: Reading %d bytes MaintenanceMode Range: %c\n", BytesRead, MaintenanceModeChar[0]);
    
    // Set offset so that we can eventually reach the end of the file
    *offset += BytesRead;
    return BytesRead;    
}

static ssize_t write_proc_maintenancemode(
    struct file *filp, 
    const char __user *buffer,
    size_t length, 
    loff_t *offset)
{
    int Ret;
    unsigned long long MaintenanceModeBuffer;
    __u8 Tmp;

    Ret = kstrtoull_from_user(buffer, length, 10, &MaintenanceModeBuffer);
    if(Ret) 
    {
        pr_debug("conegx: Error converting Maintenance Mode. RetVal = %d\n", Ret);
        return Ret;
    } 
    
    if(!(MaintenanceModeBuffer == 1 || MaintenanceModeBuffer == 0))
    {
        pr_debug("conegx: Received invalid value for Maintenance Mode: %d\n", (int)MaintenanceModeBuffer);
        return -1;
    }

    mutex_lock(&Conegx->lock);    
    pr_debug("conegx: Setting Maintenance Mode = %d\n", (int)MaintenanceModeBuffer);

    if (MaintenanceModeBuffer)
    {
        Tmp = Conegx->StatusPortBuffer | BIT_MAINTENANCE;
    }
    else
    {
        Tmp = Conegx->StatusPortBuffer & ~(BIT_MAINTENANCE);
    }
    
    if (Tmp != Conegx->StatusPortBuffer)
    {
        pr_debug("conegx: Setting Status Register to: %d \n", Tmp);
        Ret = regmap_write(Conegx->regmap, SET_STATUS_PORT, Tmp);
        
        if(Ret) 
        {
            printk(KERN_ERR "conegx: Error writing to Register SET_STATUS_PORT!\n");

            reset_MSP430();

            mutex_unlock(&Conegx->lock);
            return -1;
        }

        Conegx->StatusPortBuffer = Tmp;
    }

    mutex_unlock(&Conegx->lock);
    *offset = length;
    return length;
}

static ssize_t read_proc_resetmsp(
    struct file *filp, 
    char __user *buffer,
    size_t length, 
    loff_t *offset)
{
    mutex_lock(&Conegx->lock);
    reset_MSP430();
    mutex_unlock(&Conegx->lock);
    return 0;
}

static ssize_t write_proc_resetleaflet(
    struct file *filp, 
    const char __user *buffer,
    size_t length, 
    loff_t *offset)
{
    int Ret;
    char input[10];

    if (length >= 10)
    {
        return -EINVAL;
    }

    memset(input, 0, sizeof(input));

    if (copy_from_user(input, buffer, length))
    {
        return -EFAULT;
    }

    if (strncmp(input, "factory", 7) == 0)
    {
        pr_debug("conegx: Received signal to trigger factory reset\n");
        Ret = regmap_write(Conegx->regmap, SET_RESET, FACTORY_RESET);
        
        if(Ret) 
        {
            printk(KERN_ERR "conegx: Error writing to Register SET_RESET!\n");

            reset_MSP430();

            return -EIO;
        }
    }
    else
    {
        pr_debug("conegx: Received invalid string in /proc/conegx/resetleaflet: %s\n", input);
        return -EINVAL;
    }

    return length;
}

/**
 * @brief Read Function  for /proc/conegx/tstbuttonlock
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
    int TstButtonLock;

    mutex_lock(&Conegx->lock);
    TstButtonLock = (Conegx->StatusPortBuffer & BIT_TSTBTN_LOCK) ? 1 : 0;
    mutex_unlock(&Conegx->lock);

    TstButtonLockChar[0] = (char)(TstButtonLock + '0');
    TstButtonLockChar[1] = '\n';
    
    // If we are at the end of the file, STOP READING!
    if(BytesToRead == 0) 
    {
        return BytesToRead;
    }

    // Get bytes read by subtracting return of copy_to_user
    BytesRead = BytesToRead - copy_to_user(buffer, TstButtonLockChar + *offset, BytesToRead);
    printk("conegx: Reading %d bytes Test Button Lock. Range: %c\n", BytesRead, TstButtonLockChar[0]);

    // Set offset so that we can eventually reach the end of the file
    *offset += BytesRead;
    return BytesRead;
}

/**
 * @brief Write Function  for /proc/conegx/tstbuttonlock
 */
static ssize_t write_proc_tstbuttonlock(
    struct file *filp,
    const char *buff,
    size_t len,
    loff_t *off) 
{
    int Ret;
    unsigned long long TstButtonLockBuffer;
    __u8 Tmp;

    Ret = kstrtoull_from_user(buff, len, 10, &TstButtonLockBuffer);
    if(Ret) 
    {
        /* Negative error code. */
        pr_debug("conegx: Error converting ButtonLock. RetVal = %d\n", Ret);
        return Ret;
    } 
    
    /* Check if Value is in Range */
    if(!(TstButtonLockBuffer == 1 || TstButtonLockBuffer == 0)) 
    {
        pr_debug("conegx: Received invalid value for Test Button Lock: %d\n", (int)TstButtonLockBuffer);
        return -1;
    }

    mutex_lock(&Conegx->lock);
    /* Set Button Lock for Tst button */
    pr_debug("conegx: Setting Test Button Lock = %d\n", (int)TstButtonLockBuffer);

    if (TstButtonLockBuffer)
    {
        Tmp = Conegx->StatusPortBuffer | BIT_TSTBTN_LOCK;
    }
    else
    {
        Tmp = Conegx->StatusPortBuffer & ~(BIT_TSTBTN_LOCK);
    }
    
    if (Tmp != Conegx->StatusPortBuffer)
    {
        pr_debug("conegx: Setting Status Register to: %d \n", Tmp);
        Ret = regmap_write(Conegx->regmap, SET_STATUS_PORT, Tmp);
        
        if(Ret) 
        {
            printk(KERN_ERR "conegx: Error writing to Register SET_STATUS_PORT!\n");

            reset_MSP430();

            mutex_unlock(&Conegx->lock);
            return -1;
        }

        Conegx->StatusPortBuffer = Tmp;
    }

    mutex_unlock(&Conegx->lock);
    *off = len;
    return len;
}

/**
 * @brief Read Function for /proc/conegx/rstbuttonlock
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
    int RstButtonLock;

    mutex_lock(&Conegx->lock);
    RstButtonLock = (Conegx->StatusPortBuffer & BIT_RSTBTN_LOCK) ? 1 : 0;
    mutex_unlock(&Conegx->lock);

    RstButtonLockChar[0] = (char)(RstButtonLock + '0');
    RstButtonLockChar[1] = '\n';

    // If we are at the end of the file, STOP READING!
    if(BytesToRead == 0) 
    {
        return BytesToRead;
    }

    // Get bytes read by subtracting return of copy_to_user
    BytesRead = BytesToRead - copy_to_user(buffer, RstButtonLockChar + *offset,BytesToRead);
    printk("conegx: Reading %d bytes Reset Button Lock. Range: %c\n", BytesRead, RstButtonLockChar[0]);

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
    __u8 Tmp;

    Ret = kstrtoull_from_user(buff, len, 10, &RstButtonLockBuffer);
    if(Ret) 
    {
        /* Negative error code. */
        pr_debug("conegx: Error converting ButtonLock. RetVal = %d\n", Ret);
        return Ret;
    } 
    
    /* Check if Value is in Range */
    if(!(RstButtonLockBuffer == 1 || RstButtonLockBuffer == 0))
    {
        pr_debug("conegx: Received invalid value for Reset Button Lock: %d\n", (int)RstButtonLockBuffer);
        return -1;
    }

    mutex_lock(&Conegx->lock);
    /* Set Button Lock for Rst button */
    pr_debug("conegx: Setting Reset Button Lock = %d\n", (int)RstButtonLockBuffer);
    
    if (RstButtonLockBuffer)
    {
        Tmp = Conegx->StatusPortBuffer | BIT_RSTBTN_LOCK;
    }
    else
    {
        Tmp = Conegx->StatusPortBuffer & ~(BIT_RSTBTN_LOCK);
    }
    
    if (Tmp != Conegx->StatusPortBuffer)
    {
        pr_debug("conegx: Setting Status Register to: %d \n", Tmp);
        Ret = regmap_write(Conegx->regmap, SET_STATUS_PORT, Tmp);
        
        if(Ret) 
        {
            printk(KERN_ERR "conegx: Error writing to Register SET_STATUS_PORT!\n");

            reset_MSP430();

            mutex_unlock(&Conegx->lock);
            return -1;
        }

        Conegx->StatusPortBuffer = Tmp;
    }

    mutex_unlock(&Conegx->lock);
    *off = len;
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
    uint8_t Bit;
    
    /* Read Alert Register */
    Ret = regmap_read(Conegx->regmap, ALERT, &IrqNumber);
    
    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error reading ALERT REGISTER in IRQ handler!\n");
        
        mutex_lock(&Conegx->lock);
        reset_MSP430();
        mutex_unlock(&Conegx->lock);

        return Ret;
    }

    pr_debug("conegx: IRQ detected. Interrupt Nr.: %d\n", IrqNumber);

    /* GPIO INTERRUPTS -------------------*/
    if((IrqNumber >= POTENTIAL_FREE_INPUT_1_RISING_EDGE) 
    && (IrqNumber <= POTENTIAL_FREE_INPUT_4_FALLING_EDGE))
    {
        /* Get Gpio Number and Edge from IRQ Number */
        GpioNumber = conegx_gpio_irq_map[IrqNumber - POTENTIAL_FREE_INPUT_1_RISING_EDGE][0];

        Edge = conegx_gpio_irq_map[IrqNumber - POTENTIAL_FREE_INPUT_1_RISING_EDGE][1];

        Bit = conegx_gpio_irq_map[IrqNumber - POTENTIAL_FREE_INPUT_1_RISING_EDGE][2];

        if(Edge == RISING_EDGE) 
        {
            pr_debug("conegx: Rising Edge on %s\n", conegx_gpio_names[GpioNumber]);
            mutex_lock(&Conegx->lock);
            Conegx->InputPortBuffer |= Bit;
            mutex_unlock(&Conegx->lock);
        } 
        else if (Edge == FALLING_EDGE)
        {
            pr_debug("conegx: Falling Edge on %s\n", conegx_gpio_names[GpioNumber]);
            mutex_lock(&Conegx->lock);
            Conegx->InputPortBuffer &= ~(Bit);
            mutex_unlock(&Conegx->lock);
        }
   
        /* Trigger nested IRQ for GPIOS */
        ChildIRQ = irq_find_mapping(Conegx->chip.irq.domain, GpioNumber);
        pr_debug("conegx: handling childirq %d\n", ChildIRQ);
        handle_nested_irq(ChildIRQ);
    }
    /* WATCHDOG INTERRUPT -------------------*/
    else if(IrqNumber == WATCHDOG_RESET)
    {
        /**
         * @note Watchdog Interrupt
         * 
         * This means that the watchdog timer of the MSP430 has not been
         * reset by the Conegx Firmware and therefore caused a reset. 
         * We need to sync the register states with the Firmware.
         */

        pr_debug("conegx: Watchdog Timer interrupt occured.\n");

        Ret = handleReset();

        if(Ret)
        {
            printk(KERN_ERR "conegx: Error handling Watchdog Timer interrupt...");
            
            mutex_lock(&Conegx->lock);
            reset_MSP430();
            mutex_unlock(&Conegx->lock);
        }
    }
    else if (IrqNumber == POWER_ON_RESET)
    {
        /**
         * @note PowerOn Reset Interrupt
         * 
         * This means that the MSP430 has seen a power on reset.
         * If this happens during runtime, we need to sync the 
         * register states with the Firmware.
         */

        pr_debug("conegx: PowerOn Reset interrupt occured.\n");

        Ret = handleReset();

        if(Ret)
        {
            printk(KERN_ERR "conegx: Error handling PowerOn Reset interrupt...");
            
            mutex_lock(&Conegx->lock);
            reset_MSP430();
            mutex_unlock(&Conegx->lock);
        }        
    }
    else if(IrqNumber == I2C_EXPANDER_HARDWARE_MALFUNCTION)
    {
        printk(KERN_ERR "conegx: I2C Expander Hardware Malfunction\n");
    }
    else if(IrqNumber == RESET_BUTTON_PRESSED)
    {
        pr_debug("conegx: Reset button pressed\n");
        mutex_lock(&Conegx->lock);
        Conegx->InputPortBuffer &= ~(BIT_RESET_BUTTON);
        mutex_unlock(&Conegx->lock);
    }
    else if(IrqNumber == RESET_BUTTON_RELEASED)
    {
        pr_debug("conegx: Reset button released\n");
        mutex_lock(&Conegx->lock);
        Conegx->InputPortBuffer |= BIT_RESET_BUTTON;
        mutex_unlock(&Conegx->lock);
    }
    else if(IrqNumber == TEST_BUTTON_PRESSED)
    {
        pr_debug("conegx: Test button pressed\n");
        mutex_lock(&Conegx->lock);
        Conegx->InputPortBuffer &= ~(BIT_TEST_BUTTON);
        mutex_unlock(&Conegx->lock);
    }
    else if(IrqNumber == TEST_BUTTON_RELEASED)
    {
        pr_debug("conegx: Test button released\n");
        mutex_lock(&Conegx->lock);
        Conegx->InputPortBuffer |= BIT_TEST_BUTTON;
        mutex_unlock(&Conegx->lock);
    }
    else if(IrqNumber >= NUMBER_OF_CONEGX_IRQS)
    {
        pr_info("conegx: Received unknown IRQ number: %d", IrqNumber);

        mutex_lock(&Conegx->lock);
        reset_MSP430();
        mutex_unlock(&Conegx->lock);
        return -1;
    }

    /* Check if any IRQ is enabled and wake up Sleeping Queue */
    mutex_lock(&Conegx->lock);
    Conegx->LastInterruptNr = IrqNumber;

    if(Conegx->IRQDeviceFileEnabled) 
    {
        InterruptArrived += 1;
        wake_up(&IrSleepingQeue);
    }
    mutex_unlock(&Conegx->lock);

    return IRQ_HANDLED;
}

/**
 * @brief handle either a watchdog or a power on reset
 * 
 * @return int returns -1 if handling the reset fails, 0 if success
 */
static int handleReset(void)
{
    int Ret;
    unsigned int Val;

    /* Set OS Ready flag ----------------------------------------------------*/
    pr_debug("conegx: Setting OS Ready...\n");
    Ret = regmap_write(Conegx->regmap, SET_OS_READY, 0x1);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error setting OS Ready flag while handling reset...");
        return -1;
    }

    /* Set Relays and LEDs. */
    pr_debug("conegx: Setting relays and LEDs...\n");
    mutex_lock(&Conegx->lock);
    Ret = regmap_write(Conegx->regmap, SET_RELAY_PORT, Conegx->RelayPortBuffer);
    if(Ret)
    {
        printk(KERN_ERR "conegx: Error setting relay port while handling reset...");
        mutex_unlock(&Conegx->lock);
        return -1;
    }

    Ret = regmap_write(Conegx->regmap, SET_LED_PORT_0, Conegx->LedPort0Buffer);
    if(Ret)
    {
        printk(KERN_ERR "conegx: Error setting led port 0 while handling reset...");
        mutex_unlock(&Conegx->lock);
        return -1;
    }

    Ret = regmap_write(Conegx->regmap, SET_LED_PORT_1, Conegx->LedPort1Buffer);
    if(Ret)
    {
        printk(KERN_ERR "conegx: Error setting led port 1 while handling reset...");
        mutex_unlock(&Conegx->lock);
        return -1;
    }

    Ret = regmap_write(Conegx->regmap, SET_STATUS_PORT, Conegx->StatusPortBuffer);
    if(Ret)
    {
        printk(KERN_ERR "conegx: Error setting status port while handling reset...");
        mutex_unlock(&Conegx->lock);
        return -1;
    }

    Ret = regmap_read(Conegx->regmap, GET_INPUT_PORT, &Val);
    if(Ret)
    {
        printk(KERN_ERR "conegx: Error reading input port while handling reset...");
        mutex_unlock(&Conegx->lock);
        return -1;
    }
    Conegx->InputPortBuffer = (__u8)(Val & 0xFF);

    mutex_unlock(&Conegx->lock);
    pr_debug("conegx: Reset handled successfully.\n");

    return 0;
}

/* LED -----------------------------------------------------------------------*/

/**
 * @brief Led Set Brightness Function
 */
static int conegxled_set_brightness(
    struct led_classdev *led_cdev,
    enum led_brightness value) 
{
    int Ret;
    __u8 LedPortTmp;
    struct conegx_led *led = ldev_to_led(led_cdev);
    
    if(IO_LED_1 <= led->led_no && led->led_no <= IO_LED_6) 
    {
        mutex_lock(&Conegx->lock);
        LedPortTmp = Conegx->LedPort0Buffer;

        if (value)
        {
            pr_debug("conegx: Turn ON LED Number: %d %s\n", led->led_no, led->name);
            LedPortTmp |= BIT(led->led_no);
        }
        else
        {
            pr_debug("conegx: Turn OFF LED Number: %d %s\n", led->led_no, led->name);
            LedPortTmp &= ~(BIT(led->led_no));
        }

        if (LedPortTmp != Conegx->LedPort0Buffer)
        {
            pr_debug("conegx: Writing Register 0x%x: 0x%x\n", SET_LED_PORT_0, LedPortTmp);
            Ret = regmap_write(Conegx->regmap, SET_LED_PORT_0, LedPortTmp);

            if (Ret)
            {
                printk(KERN_ERR "conegx: Error writing to Register 0x%x\n", SET_LED_PORT_0);

                reset_MSP430();

                mutex_unlock(&Conegx->lock);
                return Ret;
            }

            Conegx->LedPort0Buffer = LedPortTmp;
        }

        mutex_unlock(&Conegx->lock);
    } 
    else if(IO_RGBLED_1_1 <= led->led_no && led->led_no <= IO_RGBLED_1_3) 
    {
        mutex_lock(&Conegx->lock);
        LedPortTmp = Conegx->LedPort1Buffer;

        if (value)
        {
            pr_debug("conegx: Turn ON RGB LED Number: %d %s\n", led->led_no, led->name);
            LedPortTmp |= BIT(led->led_no - IO_RGBLED_1_1);
        }
        else
        {
            pr_debug("conegx: Turn OFF RGB LED Number: %d %s\n", led->led_no, led->name);
            LedPortTmp &= ~(BIT(led->led_no - IO_RGBLED_1_1));
        }

        if (LedPortTmp != Conegx->LedPort1Buffer)
        {
            pr_debug("conegx: Writing Register 0x%x: 0x%x\n", SET_LED_PORT_1, LedPortTmp);
            Ret = regmap_write(Conegx->regmap, SET_LED_PORT_1, LedPortTmp);

            if (Ret)
            {
                printk(KERN_ERR "conegx: Error writing to Register 0x%x\n", SET_LED_PORT_1);

                reset_MSP430();

                mutex_unlock(&Conegx->lock);
                return Ret;
            }

            Conegx->LedPort1Buffer = LedPortTmp;
        }

        mutex_unlock(&Conegx->lock);   
    }
    else
    {
        pr_debug("conegx: Cannot set LED %d\n", led->led_no);
        return -1;
    }
    
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

    pr_debug("conegx: Setting up Leds\n");

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
            dev_err(&client->dev, "couldn't register LED %s\n", Led->ldev.name);
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
    unsigned int Val;
    unsigned int FwVersionMaj;
    unsigned int FwVersionMin;
    unsigned int FwVersionPatch;
    int TstButtonLock;
    int RstButtonLock;
    int MaintenanceMode;
    
    pr_debug("conegx: Collecting Device Infos:\n");

    /* Reading GPIO and LED States into buffers Register to identify chip */

    mutex_lock(&Conegx->lock);

    /* Get Input Port */
    Ret = regmap_read(Conegx->regmap, GET_INPUT_PORT, &Val);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read GET_INPUT_PORT Register\n");
        mutex_unlock(&Conegx->lock);
        return Ret;
    }
    Conegx->InputPortBuffer = (__u8)(Val & 0xFF);
    pr_debug("conegx: GET_INPUT_PORT: 0x%x\n", Conegx->InputPortBuffer);
    
    /* Get Relay Port */
    Ret = regmap_read(Conegx->regmap, GET_RELAY_PORT, &Val);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read GET_RELAY_PORT Register\n");
        mutex_unlock(&Conegx->lock);
        return Ret;
    } 
    Conegx->RelayPortBuffer = (__u8)(Val & 0xFF);
    pr_debug("conegx: GET_RELAY_PORT: 0x%x\n", Conegx->RelayPortBuffer);
    
    /* Get LED Port 0 */
    Ret = regmap_read(Conegx->regmap, GET_LED_PORT_0, &Val);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read GET_LED_PORT_0 Register\n");
        mutex_unlock(&Conegx->lock);
        return Ret;        
    } 
    Conegx->LedPort0Buffer = (__u8)(Val & 0xFF);
    /* In case LEDs were read during blinking sequence after startup of MSP */
    /* set PWR LED */
    Conegx->LedPort0Buffer |= BIT(IO_LED_2);
    /* reset Relay-LEDs and TLS-LED */
    Conegx->LedPort0Buffer &= ~(BIT(IO_LED_1) 
                                | BIT(IO_LED_3)
                                | BIT(IO_LED_4)
                                | BIT(IO_LED_5)
                                | BIT(IO_LED_6));
    /* TODO: decouple initial blink sequence from LED port. */
    pr_debug("conegx: GET_LED_PORT_0: 0x%x\n", Conegx->LedPort0Buffer);
    
    /* Get LED Port 1 */
    Ret = regmap_read(Conegx->regmap, GET_LED_PORT_1, &Val);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read GET_LED_PORT_1 Register\n");
        mutex_unlock(&Conegx->lock);
        return Ret; 
    } 
    Conegx->LedPort1Buffer = (__u8)(Val & 0xFF);
    pr_debug("conegx: GET_LED_PORT_1: 0x%x\n", Conegx->LedPort1Buffer);
    
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
    sprintf(Conegx->FwVersion, "%d.%d.%d\n", FwVersionMaj, FwVersionMin, FwVersionPatch);
    pr_info("conegx: FirmwareVersion: %s", Conegx->FwVersion);

    /* Get Status Port */
    Ret = regmap_read(Conegx->regmap, GET_STATUS_PORT, &Val);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read GET_STATUS_PORT Register\n");
        mutex_unlock(&Conegx->lock);
        return Ret;
    }
    Conegx->StatusPortBuffer = (__u8)(Val & 0xFF);
    pr_debug("conegx: Status port: 0x%x\n", Conegx->StatusPortBuffer);
    
    TstButtonLock = (Conegx->StatusPortBuffer & BIT_TSTBTN_LOCK) ? 1 : 0;
    RstButtonLock = (Conegx->StatusPortBuffer & BIT_RSTBTN_LOCK) ? 1 : 0;
    MaintenanceMode = (Conegx->StatusPortBuffer & BIT_MAINTENANCE) ? 1 : 0;

    pr_debug("conegx: RstButtonLock: %d\n", RstButtonLock);
    pr_debug("conegx: TstButtonLock: %d\n", TstButtonLock);
    pr_debug("conegx: Maintenance Mode: %d\n", MaintenanceMode);

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
    unsigned long IrqFlags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;

    pr_debug("conegx: Loaded in debug mode");
    pr_debug("conegx: runnning probe for %s @ 0x%x", client->name, client->addr);

    pr_info("conegx: Driver Version: %s",DRIVER_VERSION);
    
    Conegx = devm_kzalloc(&client->dev, sizeof(*Conegx), GFP_KERNEL);

    if(!Conegx)
    {
        printk(KERN_ERR "conegx: can't allocate managed device\n");
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
    Conegx->chip.ngpio = NUMBER_OF_CONEGX_GPIOS;
    Conegx->chip.can_sleep = true;

    Ret = devm_gpiochip_add_data(Conegx->dev, &Conegx->chip, Conegx);
    if(Ret < 0) 
    {
        printk(KERN_ERR "conegx: can't add GPIO chip\n");
        return Ret;
    }

    mutex_init(&Conegx->lock);

    /* add Data to I2c and GPIO */
    i2c_set_clientdata(client, Conegx);

    /* IRQ -----------------------------------------------------------------*/

    init_waitqueue_head(&IrSleepingQeue);

    /**
     * @brief allocate an interrupt line for a managed device
     * 
     * Except for the extra argument, this function takes the same arguments 
     * and performs the same function as request_irq(). 
     * 
     * IRQs requested with this function will be automatically freed on driver detach.
     * 
     * If an IRQ allocated with this function needs to be freed separately, 
     * devm_free_irq() must be used.
     * 
     * @param dev device to request interrupt for
     * @param irq Interrupt line to allocate
     * @param handler Function to be called when the IRQ occurs
     * @param thread_fn function to be called in a threaded interrupt context.
     * @param irqflags Interrupt type flags
     * @param devname An ascii name for the claiming device
     * @param dev_id A cookie passed back to the handler function
     * 
     * @return 0 if successfull, other value in case of error.
     */
    Ret = devm_request_threaded_irq(
        Conegx->chip.parent, //dev
        Conegx->irq,         //irq
        NULL,                //handler
        conegx_irq,          //thread_fn
        IrqFlags,            //irqflags
        "conegxirq",         //devname
        Conegx);             //dev_id

    if(Ret)
    {
        dev_err(
            Conegx->dev, 
            "conegx: unable to request IRQ#%d: %d\n",
            Conegx->irq, 
            Ret);

        return Ret;
    }   
    pr_debug("conegx: registered IRQ # %d\n", Conegx->irq);
    
    /* Setting up GPIO IRQ */
    Err = gpiochip_irqchip_add_nested(
        &Conegx->chip,
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

    gpiochip_set_nested_irqchip(
        &Conegx->chip,
        &Conegx->irq_chip,
        Conegx->irq);

    /* PROCFS ---------------------------------------------------------------*/
    ProcfsParent = proc_mkdir("conegx", NULL);
    if(ProcfsParent == NULL) 
    {
        printk(KERN_ERR "conegx: Error creating proc entry!\n");
        return -1;
    }

    /*Creating Proc entry under "/proc/etx/" */
    proc_create("fwversion", 0444, ProcfsParent, &proc_fops_fwversion);
    proc_create("tstbuttonlock", 0666, ProcfsParent, &proc_fops_tstbuttonlock);
    proc_create("rstbuttonlock", 0666, ProcfsParent, &proc_fops_rstbuttonlock);
    proc_create("resetmsp", 0444, ProcfsParent, &proc_fops_resetmsp);
    proc_create("maintenance", 0666, ProcfsParent, &proc_fops_maintenance);
    proc_create("resetleaflet", 0222, ProcfsParent, &proc_fops_resetleaflet);

    /* LEDS -----------------------------------------------------------------*/
    Ret = setup_leds(client);
    if (Ret)
    {
        printk(KERN_ERR "conegx: Error setting up leds!\n");
        return -1;
    }

    Ret = alloc_chrdev_region(&ConDevNr, 0, 1, "conegx_device");
    if(Ret)
    {
        printk(KERN_ERR "conegx: Error registering char device number!\n");
        return Ret;
    }

    /* Anmeldeobjekt reservieren */
    ConDriverObject = cdev_alloc(); 
    if(ConDriverObject == NULL)
    {
        unregister_chrdev_region(ConDevNr, 1);
        return -EIO;
    }

    ConDriverObject->owner = THIS_MODULE;
    ConDriverObject->ops = &fops_devfile;

    Ret = cdev_add(ConDriverObject, ConDevNr, 1);
    if(Ret)
    {
        kobject_put(&ConDriverObject->kobj);
        return -1;
    }

    ConDevClass = class_create(THIS_MODULE, "conegx_class");
    if(IS_ERR(ConDevClass)) 
    {
        pr_err("conegx_class: no udev support\n");
        kobject_put(&ConDriverObject->kobj);
        return -1;
    }

    ConDevice = device_create(ConDevClass, NULL, ConDevNr, NULL, "%s", "conegx");
    if(IS_ERR(ConDevice))
    {
        class_destroy(ConDevClass);
        return -1;
    }

    /* Reading Device Description Register to identify chip*/
    /* TODO: clean up in case of error! */
    Ret = regmap_read(Conegx->regmap, DEVICE_DESCRIPTION, &Val);
    if(Ret) 
    {
        printk(KERN_ERR "conegx: can't read DEVICE_DESCRIPTION Register\n");
        reset_MSP430();
        return Ret;
    }

    if(Val != 0x94) 
    {
        printk(KERN_ERR "conegx: DEVICE_DESCRIPTION wrong: 0x%x\n", Val);
        return -1;
    }    
    pr_debug("conegx: valid DEVICE_DESCRIPTION 0x94!\n");

    /* Read registers the first time */
    Ret = conegx_getRegister();
    if(Ret) 
    {
        printk(KERN_ERR "conegx: Error getting Device Data!\n");
        return Ret;
    }

    /* Set OS Ready flag ----------------------------------------------------*/   
    pr_debug("conegx: Setting OS Ready Flag\n");
    Ret = regmap_write(Conegx->regmap, SET_OS_READY, 0x1);
    if (Ret) 
    {
        printk(KERN_ERR "conegx: Error writing to SET_OS_READY\n");
        reset_MSP430();
        return Ret;
    }

    /* Turn On Power LED */
    pr_debug("conegx: Turning On Power LED\n");
    Ret = regmap_write(Conegx->regmap, SET_LED_PORT_0, BIT_LED_PWR);
    if (Ret)
    {
        printk(KERN_ERR "conegx: Error turning on Power LED\n");
        reset_MSP430();
        return Ret;
    }
    
    pr_info("conegx: Device Initialzed successfully\n");

    return 0;
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

    /* Remove proc entries */
    remove_proc_entry("fwversion", ProcfsParent);
    remove_proc_entry("tstbuttonlock", ProcfsParent);
    remove_proc_entry("rstbuttonlock", ProcfsParent);
    remove_proc_entry("resetmsp", ProcfsParent);
    remove_proc_entry("maintenance", ProcfsParent);
    remove_proc_entry("resetleaflet", ProcfsParent);
    remove_proc_entry("conegx", NULL);
    proc_remove(ProcfsParent);

    /* Free IRQ */
    devm_free_irq(Conegx->dev, Conegx->irq, Conegx);

    /* Destroy device */
    unregister_leds(NR_OF_LEDS);
    mutex_destroy(&Conegx->lock);
    device_destroy(ConDevClass, ConDevNr);
    class_destroy(ConDevClass);

    /* Unregister char device */
    cdev_del(ConDriverObject);
    unregister_chrdev_region(ConDevNr, 1);

    pr_debug("conegx: Device removed successfully\n");

    return 0;
}

static int reset_MSP430(void)
{
    int Ret;
    int retries = 0;
    int MAX_RETRIES = 5;
    unsigned int Val;

    pr_info("conegx: Resetting MSP430...");

    while(retries < MAX_RETRIES)
    {
        Ret = gpio_request(RST_PIN, "MSP430_Reset");
        if(Ret)
        {
            gpio_free(RST_PIN);
            mdelay(100);
            retries++;
        }
        else
        {
            Ret = gpio_direction_output(RST_PIN, 1);
            if(Ret)
            {
                gpio_free(RST_PIN);
                mdelay(100);
                retries++;
            } 
            else
            {
                retries = 0;
                break;
            }           
        }
    }
    
    if(retries != 0)
    {
        printk(KERN_ERR "conegx: Error requesting reset pin!");
        printk(KERN_ERR "conegx: Resetting MSP430 failed!");
        
        return -1;
    }

    while(retries < MAX_RETRIES)
    {
        /* Toggle Reset Pin. */
        gpio_set_value(RST_PIN, 0);

        mdelay(100);

        gpio_set_value(RST_PIN, 1);

        mdelay(100); 

        /* Set OS Ready flag ----------------------------------------------------*/
        Ret = regmap_write(Conegx->regmap, SET_OS_READY, 0x1);       
        if(Ret) 
        {
            pr_info("conegx: Error setting OS_READY flag...\n");
            retries++;
            continue;
        }     
        
        /* Set Relays and LEDs. */
        Ret = regmap_write(Conegx->regmap, SET_RELAY_PORT, Conegx->RelayPortBuffer);
        if(Ret)
        {
            printk(KERN_ERR "conegx: Error setting relay port...");
            retries++;
            continue;
        }

        Ret = regmap_write(Conegx->regmap, SET_LED_PORT_0, Conegx->LedPort0Buffer);
        if(Ret)
        {
            printk(KERN_ERR "conegx: Error setting led port 0...");
            retries++;
            continue;
        }

        Ret = regmap_write(Conegx->regmap, SET_LED_PORT_1, Conegx->LedPort1Buffer);
        if(Ret)
        {
            printk(KERN_ERR "conegx: Error setting led port 1...");
            retries++;
            continue;
        }

        Ret = regmap_write(Conegx->regmap, SET_STATUS_PORT, Conegx->StatusPortBuffer);
        if(Ret)
        {
            printk(KERN_ERR "conegx: Error setting status port...");
            retries++;
            continue;
        }

        Ret = regmap_read(Conegx->regmap, GET_INPUT_PORT, &Val);
        if(Ret)
        {
            printk(KERN_ERR "conegx: Error reading input port...");
            retries++;
            continue;
        }  
        Conegx->InputPortBuffer = (__u8)(Val & 0xFF);
        
        gpio_free(RST_PIN);
        pr_info("conegx: Reset successful\n");    
        return 0;
    }

    gpio_free(RST_PIN);
    printk(KERN_ERR "conegx: Resetting MSP430 failed!");
    
    return  -1;
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

MODULE_AUTHOR("Samuel Ardaya-Lieb <s.ardayalieb@consolinno.de>");
MODULE_DESCRIPTION("Driver for Consolinno Conegx");
MODULE_LICENSE("GPL v2");
