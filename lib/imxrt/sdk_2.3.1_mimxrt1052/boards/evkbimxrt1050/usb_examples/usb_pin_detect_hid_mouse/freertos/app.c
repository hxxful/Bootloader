/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "usb_host_hid.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "host_mouse.h"
#include "fsl_common.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
#include "app.h"
#if ((defined USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI))
#include "usb_phy.h"
#endif /* USB_HOST_CONFIG_EHCI */

#if ((!USB_HOST_CONFIG_KHCI) && (!USB_HOST_CONFIG_EHCI))
#error Please enable USB_HOST_CONFIG_KHCI or USB_HOST_CONFIG_EHCI in file usb_host_config.
#endif

#include "usb.h"
#include "usb_device_config.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "pin_mux.h"
#include "device_mouse.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
extern void BOARD_UsbVbusOn(uint8_t on);
extern void Device_AppInit(void);
extern void Device_AppDeinit(void);
extern void Device_AppTaskFunction(void);
extern void Host_AppInit(void);
extern void Host_AppDeinit(void);
extern void Host_AppTaskFunction(void);
extern void USB_DeviceKhciIsr(void);
extern void USB_DeviceEhciIsr(void);
extern void USB_HostKhciIsr(void);
extern void USB_HostEhciIsr(void);
extern void USB_HostClockInit(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_idPinStatus = 0;
volatile uint32_t g_idPinStatusChange = 0;
volatile uint32_t g_deviceMode = 0;
volatile USBHS_Type *ehciRegisterBase;
/*******************************************************************************
 * Code
 ******************************************************************************/
/* The function sets the cacheable memory to shareable, this suggestion is referred from chapter 2.2.1 Memory regions, types and attributes in Cortex-M7 Devices, Generic User Guide */
void BOARD_ConfigUSBMPU()
{
    /* Disable I cache and D cache */
    SCB_DisableICache();
    SCB_DisableDCache();

    /* Disable MPU */
    ARM_MPU_Disable();
    /* MPU configure:
     * Use ARM_MPU_RASR(DisableExec, AccessPermission, TypeExtField, IsShareable, IsCacheable, IsBufferable, SubRegionDisable, Size) 
     * API in core_cm7.h.
     * param DisableExec       Instruction access (XN) disable bit,0=instruction fetches enabled, 1=instruction fetches disabled.
     * param AccessPermission  Data access permissions, allows you to configure read/write access for User and Privileged mode.
     *      Use MACROS defined in core_cm7.h: ARM_MPU_AP_NONE/ARM_MPU_AP_PRIV/ARM_MPU_AP_URO/ARM_MPU_AP_FULL/ARM_MPU_AP_PRO/ARM_MPU_AP_RO
     * Combine TypeExtField/IsShareable/IsCacheable/IsBufferable to configure MPU memory access attributes.
     *  TypeExtField  IsShareable  IsCacheable  IsBufferable   Memory Attribtue    Shareability        Cache
     *     0             x           0           0             Strongly Ordered    shareable
     *     0             x           0           1              Device             shareable           
     *     0             0           1           0              Normal             not shareable   Outer and inner write through no write allocate           
     *     0             0           1           1              Normal             not shareable   Outer and inner write back no write allocate           
     *     0             1           1           0              Normal             shareable       Outer and inner write through no write allocate    
     *     0             1           1           1              Normal             shareable       Outer and inner write back no write allocate    
     *     1             0           0           0              Normal             not shareable   outer and inner noncache
     *     1             1           0           0              Normal             shareable       outer and inner noncache
     *     1             0           1           1              Normal             not shareable   outer and inner write back write/read acllocate
     *     1             1           1           1              Normal             shareable       outer and inner write back write/read acllocate
     *     2             x           0           0              Device              not shareable   
     *  Above are normal use settings, if your want to see more details or want to config different inner/outter cache policy.      
     *  please refer to Table 4-55 /4-56 in arm cortex-M7 generic user guide <dui0646b_cortex_m7_dgug.pdf>
     * param SubRegionDisable  Sub-region disable field. 0=sub-region is enabled, 1=sub-region is disabled.
     * param Size              Region size of the region to be configured. use ARM_MPU_REGION_SIZE_xxx MACRO in core_cm7.h.
     */
    MPU->RBAR = ARM_MPU_RBAR(7, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 1, 1, 1, 0, ARM_MPU_REGION_SIZE_32MB);	
    /* Enable MPU */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Enable I cache and D cache */
    SCB_EnableDCache();
    SCB_EnableICache();
}

void USB_DeviceClockInit(void)
{
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
    };
#endif
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, 480000000U);
    }
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
#endif
}
void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;
    irqNumber = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
#endif
/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}

void USB_DeviceIsrDisable(void)
{
    uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;
    irqNumber = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
#endif
/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
#endif
    DisableIRQ((IRQn_Type)irqNumber);
}
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle)
{
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    USB_DeviceEhciTaskFunction(deviceHandle);
#endif
}
#endif

void USB_HostClockInit(void)
{
#if defined(USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI > 0U)
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
    };
#endif
#if defined(USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI > 0U)
    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, 480000000U);
    }
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
#endif
}

void USB_HostIsrEnable(void)
{
    uint8_t irqNumber;
#if defined(USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI > 0U)
    uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
    irqNumber = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
#endif /* USB_HOST_CONFIG_EHCI */

/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}

void USB_HostIsrDisable(void)
{
    uint8_t irqNumber;
#if defined(USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI > 0U)
    uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
    irqNumber = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
#endif /* USB_HOST_CONFIG_EHCI */

/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#endif
    DisableIRQ((IRQn_Type)irqNumber);
}

void USB_HostTaskFn(void *param)
{
#if defined(USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI > 0U)
    USB_HostEhciTaskFunction(param);
#endif
}

/*!
 * @brief  board USB Vbus enable or not
 */
void BOARD_UsbVbusOn(uint8_t on)
{
    /* Some time delay waitfor power stable */
    for (int i = 0U; i < 1000000U; i++)
    {
        __ASM("nop");
    }
}

/*!
 * @brief get  USB id pin status
 */
uint8_t USB_GetIdPinStatus(void)
{
    return ((ehciRegisterBase->OTGSC & USBHS_OTGSC_ID_MASK) >> USBHS_OTGSC_ID_SHIFT);
}

/*!
 * @brief ehci host isr
 */
void USBHS_IRQHandler(void)
{
    if ((ehciRegisterBase->OTGSC & USBHS_OTGSC_IDIS_MASK) && (ehciRegisterBase->OTGSC & USBHS_OTGSC_IDIE_MASK))
    {
        ehciRegisterBase->OTGSC |= USBHS_OTGSC_IDIS_MASK;
        if (USB_GetIdPinStatus())
        {
            g_idPinStatus = 1;
        }
        else
        {
            g_idPinStatus = 0;
        }
        g_idPinStatusChange = 1;
    }
    else
    {
        if ((g_deviceMode == 0))
        {
            USB_HostEhciIsr();
        }
        else if ((g_deviceMode == 1))
        {
            USB_DeviceEhciIsr();
        }
        else
        {
        }
    }
   /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping 
     exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*!
 * @brief host mouse freertos task function.
 *
 * @param param   the host mouse instance pointer.
 */
void Host_AppTask(void *param)
{
    while (1)
    {
        if (g_deviceMode == 0)
        {
            Host_AppTaskFunction();
        }
    }
}

/*!
 * @brief device mouse freertos task function.
 *
 * @param param   the host mouse instance pointer.
 */
void Device_AppTask(void *param)
{
    while (1)
    {
        if (g_deviceMode == 1)
        {
            Device_AppTaskFunction();
        }
    }
}

/*!
 * @brief pin detect  task function.
 */
void Pin_DetectTaskFunction(void)
{
    if (g_idPinStatusChange == 1)
    {
        if (g_idPinStatus == 0)
        {
            Device_AppDeinit();
            g_deviceMode = 0;
            BOARD_UsbVbusOn(1);
            Host_AppInit();
        }
        else
        {
            Host_AppDeinit();
            g_deviceMode = 1;
            BOARD_UsbVbusOn(0);
            Device_AppInit();
        }
        g_idPinStatusChange = 0;
    }
}

void Pin_DetectTask(void *param)
{
    while (1)
    {
        Pin_DetectTaskFunction();
        vTaskDelay(100);
    }
}
/*!
 * @brief app initialization.
 */
/*!
 * @brief app initialization.
 */
void APP_init(void)
{
    uint32_t usbhsBaseAddrs[] = USBHS_BASE_ADDRS;

    if (CONTROLLER_ID - kUSB_ControllerEhci0 >= (sizeof(usbhsBaseAddrs) / sizeof(usbhsBaseAddrs[0])))
    {
        usb_echo("Pin detect:controller is not found!\r\n");
        return;
    }
    ehciRegisterBase = (USBHS_Type *)usbhsBaseAddrs[CONTROLLER_ID - kUSB_ControllerEhci0];
    USB_HostClockInit();
#if ((defined FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
    /* Some time delay waitfor phy ID status stable */
    for (volatile int i = 0U; i < 1000000U; i++)
    {
        __ASM("nop");
    }

    if (USB_GetIdPinStatus())
    {
        g_idPinStatus = 1;
        g_deviceMode = 1;
        BOARD_UsbVbusOn(0);
        Device_AppInit();
    }
    else
    {
        g_idPinStatus = 0;
        g_deviceMode = 0;
        BOARD_UsbVbusOn(1);
        Host_AppInit();
    }
    ehciRegisterBase->OTGSC |= USBHS_OTGSC_IDIE_MASK;
}

#if defined(__CC_ARM) || defined(__GNUC__)
int main(void)
#else
void main(void)
#endif
{
    BOARD_ConfigMPU();
    BOARD_ConfigUSBMPU();

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    APP_init();
    Pin_DetectTaskFunction();

    if (xTaskCreate(Pin_DetectTask, "pin detect task", 2000L / sizeof(portSTACK_TYPE), NULL, 5, NULL) != pdPASS)
    {
        usb_echo("create pin detect task error\r\n");
    }
    if (xTaskCreate(Device_AppTask, "usb device task", 2000L / sizeof(portSTACK_TYPE), NULL, 4, NULL) != pdPASS)
    {
        usb_echo("create usb device task error\r\n");
    }
    if (xTaskCreate(Host_AppTask, "usb host task", 2000L / sizeof(portSTACK_TYPE), NULL, 4, NULL) != pdPASS)
    {
        usb_echo("create usb host task error\r\n");
    }
    vTaskStartScheduler();

    while (1)
    {
        ;
    }
}
