#include <linux/pinctrl/consumer.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/gpio.h>

#include "lcm_drv.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
//#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>
//#include "gpio_const.h" 
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define TAG_NAME "[r61318_dsi_vdo_truly.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define PK_DBG(a, ...)

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER
//#define ONTIM_MODE   1
/*static u8 exit_sleep[] = {0x11};
static u8 display_on[] = {0x29};
static u8 sleep_in[] = {0x10};
static u8 display_off[] = {0x28};*/
//#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
//#define GPIO_LCD_VSN_PIN         (GPIO63 | 0x80000000)
//#define GPIO_LCD_VSP_PIN         (GPIO64 | 0x80000000)
//#define GPIO_LCD_RST_PIN         (GPIO146 | 0x80000000)

#define KERNEL_LCD



// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2_common(cmd, count, force_update)	lcm_util.dsi_set_cmdq_V2_common(cmd, count, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) 
#ifndef ASSERT
#define ASSERT(expr)                             \
    do {                                         \
        if(expr) break;                          \
        printk("DDP ASSERT FAILED %s, %d\n",     \
            __FILE__, __LINE__); BUG();          \
    }while(0)
#endif


struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


/*static u8 lcd_truly_r61318_cmd_1[]={0xB0,0x00};  
static u8 lcd_truly_r61318_cmd_2[]={0xB3,0x00};
static u8 lcd_truly_r61318_cmd_3[]={0xB6,0x32};  
static u8 lcd_truly_r61318_cmd_4[]={0xBC,0x01,0x00,0x04,0x04,0x00};  
static u8 lcd_truly_r61318_cmd_5[]={0x36,0x0A};  
static u8 lcd_truly_r61318_cmd_6[]={0xE3,0x01}; 
static u8 lcd_truly_r61318_cmd_7[]={0xC0,0x20,0xB2,0x0D,0x10,0x02,0x80};   
static u8 lcd_truly_r61318_cmd_8[]={0xC1,0x35,0x85,0x85,0x85,0x10,0x10,0x33};   
static u8 lcd_truly_r61318_cmd_9[]={0xC3,0x00,0x0F,0x00}; 
static u8 lcd_truly_r61318_cmd_10[]={0xC4,0xBC,0xD2,0x00};   
static u8 lcd_truly_r61318_cmd_11[]={0xC5,0x06,0x03,0x1D};
static u8 lcd_truly_r61318_cmd_12[]={0xC6,0x21};  
static u8 lcd_truly_r61318_cmd_13[]={0xC8,0x60,0x0E,0x08,0x62,0x90,0x52,0x95,0x54,0x49,0x29,0x84,0x90,0xA2,0x4A,0x29,0x45,0x15,0x42,0x48,0xA9,0xA4,0xA4,0xAA,0x85,0x0c,0x11,0xc0,0x31}; 
static u8 lcd_truly_r61318_cmd_14[]={0xCA,0x07,0x10,0x14,0x19,0x1F,0x25,0x24,0x20,0x23,0x24,0x20,0x19,0x0E,0x07,0x01,0x07,0x10,0x14,0x19,0x1F,0x25,0x24,0x20,0x23,0x24,0x20,0x19,0x0E,0x07,0x01};                                          
static u8 lcd_truly_r61318_cmd_15[]={0xCD,0x00};  							   
static u8 lcd_truly_r61318_cmd_16[]={0xE5,0x02};
static u8 lcd_truly_r61318_cmd_17[]={0xD0,0x05,0x89,0x1A};        //Power Setting of VCI, VGH, VGL
static u8 lcd_truly_r61318_cmd_18[]={0xD1,0x03};                        //Power Setting of VCL
static u8 lcd_truly_r61318_cmd_19[]={0xD2,0x01,0x1F};	 //Power Setting of External Booster 	  
static u8 lcd_truly_r61318_cmd_20[]={0xD4,0x50};         //use otp value     
static u8 lcd_truly_r61318_cmd_21[]={0xD5,0x34,0x34};    //VPLVL/VNLVL Setting
static u8 lcd_truly_r61318_cmd_22[]={0x35,0x00};    //VPLVL/VNLVL Setting
static u8 lcd_truly_r61318_cmd_23[]={0x51,0x00};  
static u8 lcd_truly_r61318_cmd_24[]={0x53,0x24};
static u8 lcd_truly_r61318_cmd_25[]={0x55,0x00}; */



static struct LCM_setting_table lcm_initialization_setting[] = {
{0xB0,1,{0x00}},  
{0xB3,1,{0x00}},
{0xB6,1,{0x32}},  
{0x36,1,{0x0A}},  
{0xC0,6,{0x20,0xB2,0x0D,0x10,0x02,0x80}},   
{0xC1,7,{0x35,0x85,0x85,0x85,0x10,0x10,0x33}},   
{0xC3,3,{0x00,0x0F,0x00}}, 
{0xC4,3,{0xBC,0xD2,0x00}},   
{0xC5,3,{0x06,0x03,0x1D}},
{0xC6,1,{0x21}},  
{0xC8,28,{0x60,0x0E,0x08,0x62,0x90,0x52,0x95,0x54,0x49,0x29,0x84,0x90,0xA2,0x4A,0x29,0x45,0x15,0x42,0x48,0xA9,0xA4,0xA4,0xAA,0x85,0x0c,0x11,0xc0,0x31}}, 
{0xCA,30,{0x02,0x0c,0x12,0x18,0x1f,0x24,0x26,0x23,0x20,0x20,0x1c,0x12,0x0f,0x08,0x01,0x02,0x0c,0x12,0x18,0x1f,0x24,0x26,0x23,0x20,0x20,0x1c,0x12,0x0f,0x08,0x01}},                                          
{0xCD,1,{0x00}},  							   
{0xE5,1,{0x02}},
{0xD0,3,{0x05,0x89,0x1A}},        //Power Setting of VCI, VGH, VGL
{0xD1,1,{0x03}},                        //Power Setting of VCL
{0xD2,2,{0x01,0x1F}},	 //Power Setting of External Booster 	  
{0xD4,1,{0x50}},         //VCOMDC Setting  80      
{0xD5,2,{0x34,0x34}},    //VPLVL/VNLVL Setting
{0x51,1,{0x00}},  
{0x53,1,{0x24}},
{0x55,1,{0x00}},  
{0x11, 0, {}},
{REGFLAG_DELAY, 120, {}},
{0x29, 0, {}},
{REGFLAG_DELAY, 20, {}},
};


/*static struct LCM_setting_table lcm_set_window[] = {
    {0x2A, 4, {0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
    {0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};*/


/*static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 0, {}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 0, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};*/


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 0, {}},
	{REGFLAG_DELAY, 20, {}},
    // Sleep Mode On
    {0x10, 0, {}},
    {REGFLAG_DELAY, 120, {}},
    {0xb0, 1, {0x00}},
    {0xb1, 1, {0x01}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*static u8 lcd_truly_r61318_cmd1[]={0xB0,0x00};  
static u8 lcd_truly_r61318_cmd2[]={0xB1,0x01};
static struct LCM_setting_table_common lcm_deep_sleep_in_setting_common[] = {
{DSI_CMD_AUTO, 20,sizeof(display_off), display_off},
{DSI_CMD_AUTO, 120,sizeof(sleep_in), sleep_in},
{DSI_CMD_AUTO, 0, sizeof(lcd_truly_r61318_cmd1),lcd_truly_r61318_cmd1},
{DSI_CMD_AUTO, 0, sizeof(lcd_truly_r61318_cmd2),lcd_truly_r61318_cmd2},
};*/

static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active				= 2;
    params->dsi.vertical_backporch					= 8;
    params->dsi.vertical_frontporch					= 15;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 50;
    params->dsi.horizontal_backporch				= 50;
    params->dsi.horizontal_frontporch				= 50;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 218; //this value must be in MTK suggested table

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
}

#ifdef KERNEL_LCD
struct pinctrl *lcd_pinctrl;
struct pinctrl_state *pinctrl_vsp_low;
struct pinctrl_state *pinctrl_vsp_high;
struct pinctrl_state *pinctrl_vsn_low;
struct pinctrl_state *pinctrl_vsn_high;

static int lcd_gpio_init(struct platform_device *pdev)
{
	int ret = 0;
printk("[kernel]:lcd_gpio_init start\n");
	lcd_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lcd_pinctrl)) {
		dev_err(&pdev->dev, "Cannot find lcd_pinctrl!");
		printk("[kernel]:Cannot find lcd_pinctrl\n");
		ret = PTR_ERR(lcd_pinctrl);
		return ret;
	}
	/*Cam0 Power/Rst Ping initialization */
	pinctrl_vsp_low = pinctrl_lookup_state(lcd_pinctrl, "vsp_low");
	if (IS_ERR(pinctrl_vsp_low)) {
		ret = PTR_ERR(pinctrl_vsp_low);
		printk("[kernel]:pinctrl err vsp_low\n");
		pr_debug("%s : pinctrl err, vsp_low\n", __func__);
		return ret;
	}

	pinctrl_vsp_high = pinctrl_lookup_state(lcd_pinctrl, "vsp_high");
	if (IS_ERR(pinctrl_vsp_high)) {
		ret = PTR_ERR(pinctrl_vsp_high);
		printk("[kernel]:pinctrl err vsp_high\n");
		pr_debug("%s : pinctrl err, vsp_high\n", __func__);
		return ret;
	}

	pinctrl_vsn_low = pinctrl_lookup_state(lcd_pinctrl, "vsn_low");
	if (IS_ERR(pinctrl_vsn_low)) {
		ret = PTR_ERR(pinctrl_vsn_low);
		printk("[kernel]:pinctrl err vsn_low\n");
		pr_debug("%s : pinctrl err, vsn_low\n", __func__);
		return ret;
	}

	pinctrl_vsn_high = pinctrl_lookup_state(lcd_pinctrl, "vsn_high");
	if (IS_ERR(pinctrl_vsn_high)) {
		ret = PTR_ERR(pinctrl_vsn_high);
		printk("[kernel]:pinctrl err vsn_high\n");
		pr_debug("%s : pinctrl err, vsn_high\n", __func__);
		return ret;
	}

	pinctrl_select_state(lcd_pinctrl, pinctrl_vsp_low);
	pinctrl_select_state(lcd_pinctrl, pinctrl_vsn_low);
	
printk("[kernel]:lcd_gpio_init end\n");
	return ret;
}



static int lcd_power_probe(struct platform_device *pdev)
{
	int ret = 0;
	printk("[kernel]:lcd_power_probe start.\n");
	pr_debug("lcd_power_probe enter\n");
	ret = lcd_gpio_init(pdev);
	printk("[kernel]:lcd_power_probe end.\n");
	return 0;
}
/*
static int strobe_remove(struct platform_device *pdev)
{
	return 0;
}

static int strobe_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int strobe_resume(struct platform_device *pdev)
{
	return 0;
}
*/
#ifdef CONFIG_OF
static const struct of_device_id lcd_of_ids[] = {
	{.compatible = "mediatek,lcd_power",},
	{}
};
#endif

static struct platform_driver Lcd_Power_Driver = {
	.probe = lcd_power_probe,
	//.remove = strobe_remove,
	//.suspend = strobe_suspend,
	//.resume = strobe_resume,
	.driver = {
		   .name = "lcd_power",
		   .owner = THIS_MODULE,

#ifdef CONFIG_OF
		   .of_match_table = lcd_of_ids,
#endif

		   }
};

static int __init lcd_power_init(void)
{
	printk("[kernel]:lcd_power_init start.\n");
	PK_DBG("Lcd_Power_Driver\n");
	if (platform_driver_register(&Lcd_Power_Driver)) {
		printk("failed to register Lcd_Power_Driver\n");
		return -ENODEV;
	}
	printk("[kernel]:lcd_power_init end.\n");
	return 0;
}

static void __exit Lcd_power_exit(void)
{
	platform_driver_unregister(&Lcd_Power_Driver);
}

module_init(lcd_power_init);
module_exit(Lcd_power_exit);

MODULE_DESCRIPTION("Lcd_pin");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");



int Lcd_Enable(void)
{
  	pinctrl_select_state(lcd_pinctrl, pinctrl_vsp_high);
	pinctrl_select_state(lcd_pinctrl, pinctrl_vsn_high);
	printk("Lcd_Enable\n");
	return 0;
}

int Lcd_Disable(void)
{
	pinctrl_select_state(lcd_pinctrl, pinctrl_vsp_low);
	pinctrl_select_state(lcd_pinctrl, pinctrl_vsn_low);
	printk("Lcd_Disable\n");
	return 0;
}
#endif
static void lcm_reset(void)
{
	printk("[kernel]:lcm reset start.\n");
	/*mt_set_gpio_out(GPIO_LCD_VSP_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_LCD_VSN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ONE);
	MDELAY(10);*/
	
	#ifdef KERNEL_LCD
	Lcd_Enable();
	#endif
	SET_RESET_PIN(1);	
	SET_RESET_PIN(0);
	MDELAY(10); // 1ms
	SET_RESET_PIN(1);
	printk("[kernel]:lcm reset end.\n");
}

static void lcm_init(void)
{
	printk("[kernel]:r61318_lcm_init start.\n");
	lcm_reset();

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

	printk("[kernel]:r61318_lcm_init end.\n");
}
static void lcm_suspend(void)
{
	printk("[kernel]:lcm_suspend start.\n");

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

	MDELAY(20);
	/*mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ZERO);
	MDELAY(2);
	mt_set_gpio_out(GPIO_LCD_VSN_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_LCD_VSP_PIN, GPIO_OUT_ZERO);
	MDELAY(4);*/
	SET_RESET_PIN(0);
	MDELAY(4); // 1ms
#ifdef KERNEL_LCD
	Lcd_Disable();
#endif
	printk("[kernel]:lcm_suspend end.\n");
}


static void lcm_resume(void)
{
	printk("[kernel]:lcm_resume start.\n");
	lcm_init();
	printk("[kernel]:lcm_resume end.\n");
}


//static u8 lcd_backlight_control_cmd[]={0x51,0x00};
/*static struct LCM_setting_table_common lcd_backlight_control_cmds[] = {
{DSI_CMD_AUTO, 0, sizeof(lcd_backlight_control_cmd),lcd_backlight_control_cmd},  
};*/
static void lcm_setbacklight_truly(unsigned int level)
{
	//int lcd_backlight_level = 0;

	//printk("[kernel]:%s,start level=%d.\n",__func__,level);
	if(level <0)
		level=0;
	else if(level > 255) 
		level = 255;

	// Refresh value of backlight level.

	lcm_backlight_level_setting[0].para_list[0] = level;
	push_table(lcm_backlight_level_setting, ARRAY_SIZE(lcm_backlight_level_setting), 1);

	printk(KERN_ERR "[kernel]:%s,lcd_backlight_level=%d.\n",__func__,level);
}


/*void r61318_write_read_cmd(void)
{
	dsi_set_cmdq_V2_common(lcd_truly_r61318_write_cmds, ARRAY_SIZE(lcd_truly_r61318_write_cmds), 1);

	printk(KERN_ERR "[kernel]:%s,end.\n",__func__);
}*/

LCM_DRIVER r61318_dsi_vdo_lcm_drv_truly = 
{
    .name			= "r61318_dsi_vdo_truly",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .set_backlight	= lcm_setbacklight_truly,
    //.lcd_id_voltage = 0,
};
