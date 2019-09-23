/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   GPIO输出—使用固件库点亮LED灯
  ******************************************************************
  * @attention
  *
  * 实验平台:野火  i.MXRT1052开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************
  */
#include "fsl_debug_console.h"

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "./led/bsp_led.h"

#include "fsl_iomuxc.h"
#include "pad_config.h"



/*******************************************************************
 * Prototypes
 *******************************************************************/
/**
 * @brief 延时一段时间
 */
static void delay(uint32_t count);

/*******************************************************************
 * Code
 *******************************************************************/
/**
 * @note 本函数在不同的优化模式下延时时间不同，
 *       如flexspi_nor_debug和flexspi_nor_release版本的程序中，
 *       flexspi_nor_release版本的延时要短得多
 */
static void delay(uint32_t count)
{
    volatile uint32_t i = 0;
    for (i = 0; i < count; ++i) {
        __asm("NOP"); /* 调用nop空指令 */
    }
}

#define BOARD_PWR_IOMUXC        IOMUXC_SNVS_PMIC_STBY_REQ_GPIO5_IO02
#define BOARD_PWR_GPIO          GPIO5
#define BOARD_PWR_GPIO_PIN      (2U)
#define BOARD_PWR_PAD_DATA      (SRE_0_SLOW_SLEW_RATE| \
                                DSE_6_R0_6| \
                                SPEED_2_MEDIUM_100MHz| \
                                ODE_0_OPEN_DRAIN_DISABLED| \
                                PKE_0_PULL_KEEPER_DISABLED| \
                                PUE_0_KEEPER_SELECTED| \
                                PUS_0_100K_OHM_PULL_DOWN| \
                                HYS_0_HYSTERESIS_DISABLED)

#define BOARD_LED_IOMUXC        IOMUXC_GPIO_EMC_36_GPIO3_IO22
#define BOARD_LED_GPIO          GPIO3
#define BOARD_LED_GPIO_PIN      (22U)
#define BOARD_LED_PAD_DATA      


void BOARD_PowerConfig(void)
{
    /* 定义gpio初始化配置结构体 */
    gpio_pin_config_t config;

    /** 底板的PWR，GPIO配置 **/
    config.direction = kGPIO_DigitalOutput; //输出模式
    config.outputLogic =  1;                //默认高电平
    config.interruptMode = kGPIO_NoIntmode; //不使用中断

    IOMUXC_SetPinMux(BOARD_PWR_IOMUXC, 0U);
    IOMUXC_SetPinConfig(BOARD_PWR_IOMUXC, 0x00B0);
    GPIO_PinInit(BOARD_PWR_GPIO, BOARD_PWR_GPIO_PIN, &config);
}

void BOARD_LedConfig(void)
{
    /* 定义gpio初始化配置结构体 */
    gpio_pin_config_t config;

    /** 底板的PWR，GPIO配置 **/
    config.direction = kGPIO_DigitalOutput; //输出模式
    config.outputLogic =  1;                //默认高电平
    config.interruptMode = kGPIO_NoIntmode; //不使用中断

    IOMUXC_SetPinMux(BOARD_LED_IOMUXC, 0U);
    IOMUXC_SetPinConfig(BOARD_LED_IOMUXC, 0x00B0);
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &config);
}

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
    /* 初始化内存保护单元 */
    BOARD_ConfigMPU();
    /* 初始化开发板引脚 */
    BOARD_PowerConfig();
    /* 初始化状态LED灯 */
    BOARD_LedConfig();
    BOARD_InitPins();
    /* 初始化开发板时钟 */
    BOARD_BootClockRUN();
    /* 初始化调试串口 */
    BOARD_InitDebugConsole();
    /* 打印系统时钟 */
    PRINTF("\r\n");
    PRINTF("*****欢迎使用 野火i.MX RT1052 开发板*****\r\n");
    PRINTF("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
    PRINTF("AHB:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_AhbClk));
    PRINTF("SEMC:            %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SemcClk));
    PRINTF("SYSPLL:          %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllClk));
    PRINTF("SYSPLLPFD0:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd0Clk));
    PRINTF("SYSPLLPFD1:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd1Clk));
    PRINTF("SYSPLLPFD2:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk));
    PRINTF("SYSPLLPFD3:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd3Clk));

    PRINTF("GPIO输出-使用固件库点亮LED\r\n");

    /* 初始化LED引脚 */
    //LED_GPIO_Config();

    while(1) {
        PRINTF("HELLO UAVRS_V2:0x%08X\r\n", BOARD_PWR_PAD_DATA);
        delay(LED_DELAY_COUNT);
    }

}


/****************************END OF FILE**********************/

void _start(void)
{
    main();
}

