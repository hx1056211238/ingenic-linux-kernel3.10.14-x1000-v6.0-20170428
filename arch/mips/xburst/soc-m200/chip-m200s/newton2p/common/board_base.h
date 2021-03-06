#ifndef __BOARD_BASE_H__
#define __BOARD_BASE_H__
#include <linux/i2c.h>

#include <board.h>

#ifdef CONFIG_KEYBOARD_GPIO
extern struct platform_device jz_button_device;
#endif
#ifdef CONFIG_TOUCHSCREEN_FT6X06
extern struct ft6x06_platform_data ft6x06_tsc_pdata;
#endif
#ifdef CONFIG_VIDEO_OVISP
extern struct ovisp_camera_platform_data ovisp_camera_info;
#endif
#ifdef CONFIG_INV_MPU_IIO
extern struct mpu_platform_data mpu9250_platform_data;
#endif
#if (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C0_V12_JZ))
extern struct i2c_board_info jz_i2c0_devs[];
extern int jz_i2c0_devs_size;
#endif
#if (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C1_V12_JZ))
extern struct i2c_board_info jz_i2c1_devs[];
extern int jz_i2c1_devs_size;
#endif
#if (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C2_V12_JZ))
extern struct i2c_board_info jz_i2c2_devs[];
extern int jz_i2c2_devs_size;
#endif
#ifdef CONFIG_I2C_GPIO
#ifndef CONFIG_I2C0_V12_JZ
extern struct platform_device i2c0_gpio_device;
#endif
#ifndef CONFIG_I2C1_V12_JZ
extern struct platform_device i2c1_gpio_device;
#endif
#ifndef CONFIG_I2C2_V12_JZ
extern struct platform_device i2c2_gpio_device;
#endif
#ifndef CONFIG_I2C3_V12_JZ
extern struct platform_device i2c3_gpio_device;
#endif
#endif	/* CONFIG_I2C_GPIO */

#ifdef CONFIG_SOUND_OSS_XBURST
extern struct snd_codec_data codec_data;
#endif
#ifdef CONFIG_BCM_PM_CORE
extern struct platform_device bcm_power_platform_device;
#endif
#ifdef CONFIG_BROADCOM_RFKILL
extern struct platform_device bt_power_device;
extern struct platform_device bluesleep_device;
#endif
#ifdef CONFIG_BCMDHD_1_141_66
extern struct platform_device wlan_device;
#endif
#ifdef CONFIG_BCM2079X_NFC
extern struct bcm2079x_platform_data bcm2079x_pdata;
#endif
#ifndef CONFIG_NAND
#ifdef CONFIG_JZMMC_V12_MMC0
extern struct jzmmc_platform_data inand_pdata;
#endif
#endif
#ifdef CONFIG_JZMMC_V12_MMC1
extern struct jzmmc_platform_data sdio_pdata;
#endif

/* Digital pulse backlight*/
#ifdef CONFIG_BACKLIGHT_DIGITAL_PULSE
extern struct platform_device digital_pulse_backlight_device;
extern struct platform_digital_pulse_backlight_data bl_data;
#endif
#ifdef CONFIG_BACKLIGHT_PWM
extern struct platform_device backlight_device;
#endif

#ifdef CONFIG_JZ_EPD_V12
extern struct platform_device jz_epd_device;
extern struct jz_epd_platform_data jz_epd_pdata;
#endif

/* lcd pdata and display panel */
#ifdef CONFIG_FB_JZ_V12
extern struct jzfb_platform_data jzfb_pdata;
#endif
#ifdef CONFIG_FB_MIPI_ONLY
extern struct jzfb_platform_data jzfb_pdata;
#endif
#ifdef CONFIG_LCD_KFM701A21_1A
extern struct platform_device kfm701a21_1a_device;
#endif
#ifdef CONFIG_LCD_LH155
extern struct mipi_dsim_lcd_device	lh155_device;
#endif
#ifdef CONFIG_LCD_BYD_9177AA
extern struct mipi_dsim_lcd_device	byd_9177aa_device;
#endif
#ifdef CONFIG_LCD_X163
extern struct mipi_dsim_lcd_device  auo_x163_device;
#endif
#ifdef CONFIG_LCD_BOE_TFT320320
extern struct mipi_dsim_lcd_device  boe_tft320320_device;
#endif
#ifdef CONFIG_LCD_TRULY_TDO_HD0499K
extern struct mipi_dsim_lcd_device	truly_tdo_hd0499k_device;
#endif
#ifdef CONFIG_LCD_EDO_E1392AM1
extern struct mipi_dsim_lcd_device  edo_e1392am1_device;
#endif
#ifdef CONFIG_LCD_CV90_M5377_P30
extern struct platform_device cv90_m5377_p30_device;
#endif
#ifdef CONFIG_LCD_BYD_BM8766U
extern struct platform_device byd_bm8766u_device;
#endif
#ifdef CONFIG_LCD_BYD_8991FTGF
extern struct platform_device byd_8991_device;
#endif
#ifdef CONFIG_LCD_TRULY_TFT240240_2_E
extern struct platform_device truly_tft240240_device;
#endif
#ifdef CONFIG_JZ_BATTERY
extern struct jz_adc_platform_data adc_platform_data;
#endif
#if IS_ENABLED(CONFIG_SND_ASOC_INGENIC_NEWTON_ICDC)
extern struct platform_device snd_newton_device;
#endif
#ifdef CONFIG_SND_ASOC_INGENIC_NEWTON2_PLUS_ICDC
extern struct platform_device snd_newton2_plus_device;
#endif

#endif	/* __BOARD_BASE_H__ */
