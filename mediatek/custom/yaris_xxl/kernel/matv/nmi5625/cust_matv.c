//For mt6573_evb
///#include <mach/mt6575_pll.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>
///#include <linux/delay.h>


#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <linux/jiffies.h>
#include <linux/timer.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_reg_base.h>

#include "cust_matv.h"
#include "cust_matv_comm.h"

#define NMITV_NAME	"nmtatv"

int cust_matv_power_on(void)
{  
	MATV_LOGE("[MATV] cust_matv_power_on Start at %s\n", NMITV_NAME);
/*	
	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,"mode_name"))//AVDD
	{
		MATV_LOGE("[CAMERA SENSOR] Fail to enable analog power\n");
		return 0;
	}
	if(mt_set_gpio_mode(GPIO_CAMERA_CMRST1_PIN,GPIO_CAMERA_CMRST1_PIN_M_GPIO)){printk("[CAMERA SENSOR] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_CMRST1_PIN,GPIO_DIR_OUT)){printk("[CAMERA SENSOR] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)){printk("[CAMERA SENSOR] set gpio failed!! \n");}
	if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){printk("[CAMERA LENS] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){printk("[CAMERA LENS] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)){printk("[CAMERA LENS] set gpio failed!! \n");}
*/
    if(TRUE != hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, NMITV_NAME))
    {
    	MATV_LOGE("[CAMERA SENSOR] Fail to enable digital power\n");
    	return 0;
    }                    
/*
    if(TRUE != hwPowerOn(MT6323_POWER_LDO_VCAM_IO, VOL_1800, NMITV_NAME))
    {
    	MATV_LOGE("[CAMERA SENSOR] Fail to enable analog power\n");
    	return 0;
    }
*/	
    return 0;
}


int cust_matv_power_off(void)
{  
	MATV_LOGE("[MATV] cust_matv_power_off Start at %s\n", NMITV_NAME);

    if(TRUE != hwPowerDown(MT6323_POWER_LDO_VGP3, NMITV_NAME)) {
        MATV_LOGE("[CAMERA SENSOR] Fail to OFF analog power\n");
        return 0;
    }
/*
    if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAM_IO, NMITV_NAME))
    {
        MATV_LOGE("[CAMERA SENSOR] Fail to enable analog power\n");
        return 0;
    }
*/	
    return 0;
}

#define TV_I2S_LRCLK	GPIO106
#define TV_I2S_SCLK		GPIO107
#define TV_I2S_DATA		GPIO105

int cust_matv_gpio_on(void)
{
	MATV_LOGE("[MATV] cust_matv_gpio_on Start at %s\n", NMITV_NAME);

    mt_set_gpio_mode(TV_I2S_DATA, GPIO_MODE_01);
    mt_set_gpio_mode(TV_I2S_SCLK, GPIO_MODE_01);
    mt_set_gpio_mode(TV_I2S_LRCLK, GPIO_MODE_01);
    return 1;
}

int cust_matv_gpio_off(void)
{
	MATV_LOGE("[MATV] cust_matv_gpio_off Start at %s\n", NMITV_NAME);

    mt_set_gpio_mode(TV_I2S_DATA, GPIO_MODE_00);
    mt_set_gpio_dir(TV_I2S_DATA,GPIO_DIR_OUT);
    mt_set_gpio_out(TV_I2S_DATA,GPIO_OUT_ZERO);
	
    mt_set_gpio_mode(TV_I2S_SCLK, GPIO_MODE_00);
    mt_set_gpio_dir(TV_I2S_SCLK,GPIO_DIR_OUT);
    mt_set_gpio_out(TV_I2S_SCLK,GPIO_OUT_ZERO);
	
    mt_set_gpio_mode(TV_I2S_LRCLK, GPIO_MODE_00);
    mt_set_gpio_dir(TV_I2S_LRCLK,GPIO_DIR_OUT);
    mt_set_gpio_out(TV_I2S_LRCLK,GPIO_OUT_ZERO);
    return 1;
}

