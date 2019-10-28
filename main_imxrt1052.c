/*
 * UAVRS_V2 board support for the bootloader.
 *
 */
#include "board.h"

#include "hw_config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "bl.h"
#include "uart.h"

#define BOOTLOADER_RESERVATION_SIZE     (24 * 1024)
#define FIRST_FLASH_SECTOR_TO_ERASE     (BOARD_FIRST_FLASH_SECTOR_TO_ERASE + (BOOTLOADER_RESERVATION_SIZE/FLASH_SECTOR_SIZE))
#define APP_SIZE_MAX                    (BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	(void *)BOARD_UART
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  	NULL
#endif

/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,
	.systick_mhz	= 120,
};

static void board_init(void);

#define BOOT_RTC_SIGNATURE          0xb007b007
#define POWER_DOWN_RTC_SIGNATURE    0xdeaddead // Written by app fw to not re-power on.
#define BOOT_RTC_REG                0x62000000

/* State of an inserted USB cable */
static bool usb_connected = false;

static uint32_t
board_get_rtc_signature()
{
	return *((volatile uint32_t *)BOOT_RTC_REG);
}

static void
board_set_rtc_signature(uint32_t sig)
{
	*((volatile uint32_t *)BOOT_RTC_REG) = sig;
}


static bool
board_test_force_pin()
{
	return false;
}

#if INTERFACE_USART == 1
static bool
board_test_usart_receiving_break()
{
	return false;
}
#endif

uint32_t
board_get_devices(void)
{
	uint32_t devices = BOOT_DEVICES_SELECTION;

	if (usb_connected) {
		devices &= BOOT_DEVICES_FILTER_ONUSB;
	}

	return devices;
}

static void
board_init(void)
{

}

void
board_deinit(void)
{

}


void
clock_deinit(void)
{

}

void flash_lock(void)
{

}

uint32_t flash_func_sector_size(unsigned sector)
{
    return 0;
}

static bool flash_verify_erase(unsigned sector)
{
    return true;
}

void flash_erase_sector(unsigned sector)
{
        flash_verify_erase(sector);
}



void flash_func_erase_sector(unsigned sector)
{
        flash_erase_sector(sector);

}


void
flash_func_write_word(uint32_t address, uint32_t word)
{

}

uint32_t flash_func_read_word(uint32_t address)
{
        return 0;
}

uint32_t
flash_func_read_otp(uint32_t address)
{
	return 0;
}

uint32_t get_mcu_id(void)
{
	return 0;
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	const char dig[] = "0123456789ABCDEF";
	const char none[] = "i.MXRT1052,0";
	int i;

	for (i = 0; none[i] && i < max - 1; i++) {
		revstr[i] = none[i];
	}

	uint32_t id = get_mcu_id();
	revstr[i - 1] = dig[id];
	return i;
}


int check_silicon(void)
{
	return 0;
}

uint32_t
flash_func_read_sn(uint32_t address)
{
	return 0;
}

void
led_on(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		//BOARD_LED_ON(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		//BOARD_LED_ON(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

void
led_off(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		//BOARD_LED_OFF(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		//BOARD_LED_OFF(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

void
led_toggle(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		//GPIO_TogglePinsOutput(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		//GPIO_TogglePinsOutput(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

/* 预防GCC编译报错 */
#ifdef USB_DATA_ALIGN
#undef USB_DATA_ALIGN
#endif

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"
#include "fsl_debug_console.h"

#include "usb_device_descriptor.h"
#include "virtual_com.h"

extern void USB_DeviceApplicationInit(void);
extern void APPTask(void);
extern usb_status_t USB_DeviceCdcAcmSend(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);
extern usb_cdc_vcom_struct_t s_cdcVcom;

int
main(void)
{
        /* 初始化内存保护单元 */
        BOARD_ConfigMPU();
        /* 初始化底板电源 */
        board_init_power();
        /* 初始化引脚 */
        BOARD_InitPins();
        /* 初始化开发板时钟 */
        BOARD_BootClockRUN();
        /* 初始化调试串口 */
        BOARD_InitDebugConsole();
        
        /* 初始化状态LED灯 */
        //board_init_led();
        
        /* 显示系统时钟信息 */
        system_clock_info();
        USB_DeviceApplicationInit();
        /* 初始化HyperFlash */
        board_init_hyperflash();
        while(1) {
		APPTask();
                USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
        }
        /* 使GCC编译时不报错 */
        board_init();
        board_get_rtc_signature();
        board_set_rtc_signature(0);
        board_test_force_pin();

#if 0
	bool try_boot = true;			/* try booting before we drop to the bootloader */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

#if defined(BOARD_POWER_PIN_OUT)

	/* Here we check for the app setting the POWER_DOWN_RTC_SIGNATURE
	 * in this case, we reset the signature and wait to die
	 */
	if (board_get_rtc_signature() == POWER_DOWN_RTC_SIGNATURE) {
		board_set_rtc_signature(0);

		while (1);
	}

#endif

	/* do board-specific initialization */
	board_init();

	/*
	 * Check the force-bootloader register; if we find the signature there, don't
	 * try booting.
	 */
	if (board_get_rtc_signature() == BOOT_RTC_SIGNATURE) {

		/*
		 * Don't even try to boot before dropping to the bootloader.
		 */
		try_boot = false;

		/*
		 * Don't drop out of the bootloader until something has been uploaded.
		 */
		timeout = 0;

		/*
		 * Clear the signature so that if someone resets us while we're
		 * in the bootloader we'll try to boot next time.
		 */
		board_set_rtc_signature(0);
	}

	/*
	 * Check if the force-bootloader pins are strapped; if strapped,
	 * don't try booting.
	 */
	if (board_test_force_pin()) {
		try_boot = false;
	}

#if INTERFACE_USB

	/*
	 * Check for USB connection - if present, don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
#if defined(BOARD_USB_VBUS_SENSE_DISABLED)
	try_boot = false;
#else

//	if (GPIO_ReadPinInput(KINETIS_GPIO(BOARD_PORT_VBUS), BOARD_PIN_VBUS) != 0) {
//		usb_connected = true;
//		/* don't try booting before we set up the bootloader */
//		try_boot = false;
//	}

#endif
#endif

#if INTERFACE_USART

	/*
	 * Check for if the USART port RX line is receiving a break command, or is being held low. If yes,
	 * don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
	if (board_test_usart_receiving_break()) {
		try_boot = false;
	}

#endif

	/* Try to boot the app if we think we should just go straight there */
	if (try_boot) {

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* try to boot immediately */
		jump_to_app();

		// If it failed to boot, reset the boot signature and stay in bootloader
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);

		/* booting failed, stay in the bootloader forever */
		timeout = 0;
	}

	/* start the interface */
#if INTERFACE_USART
	cinit(BOARD_INTERFACE_CONFIG_USART, USART);
#endif
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif

	while (1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader(timeout);

		/* if the force-bootloader pins are strapped, just loop back */
		if (board_test_force_pin()) {
			continue;
		}

#if INTERFACE_USART

		/* if the USART port RX line is still receiving a break, just loop back */
		if (board_test_usart_receiving_break()) {
			continue;
		}

#endif

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* look to see if we can boot the app */
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
#endif
}

void _start()
{
	main();
}

