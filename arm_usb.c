/*
 * KEYBOARD FOR STM32
 * Copyright (C) 2024 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */


#include "linux.h"
#include "misc.h"
#include "uart.h"
#include "usb_core.h"
#include "usb_dcd.h"
#include "usbh_core.h"
#include "usb_hcd_int.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "arm_usb.h"

// Have to disable interrupts & use handle_usb to print from the interrupt handlers
//#define ENABLE_INTERRUPT

extern int initialized;
USB_OTG_CORE_HANDLE  USB_OTG_dev;

#define USB_MAX_STR_DESC_SIZ         64 
uint8_t USBD_StrDesc[USB_MAX_STR_DESC_SIZ];

#define USB_DEVICE_DESCRIPTOR_TYPE   0x01
// vendor ID
#define USBD_VID                     0x04d8
// product ID
#define USBD_PID                     0x000e

#define LOBYTE(x)  ((uint8_t)(x & 0x00FF))
#define HIBYTE(x)  ((uint8_t)((x & 0xFF00) >>8))
#define USB_WORD(x) (uint8_t)((x) & 0xff), (uint8_t)((x) >> 8)

#define  USBD_IDX_MFC_STR                               0x01 
#define  USBD_IDX_PRODUCT_STR                           0x02
#define  USBD_IDX_SERIAL_STR                            0x03 
#define USBD_CFG_MAX_NUM           1
#define _EP_IN 0x80
#define _EP_OUT 0x00
#define CDC_EP  0x01
#define DATA_EP 0x02
#define HID_EP 0x01
//#define HID_IN_PACKET 16
#define HID_IN_PACKET 8
#define EP1_IN_ID      (_EP_IN | HID_EP)
#define EP1_OUT_ID      0x01

#define EP1_SIZE HID_IN_PACKET




/* USB Standard Device Descriptor */
const uint8_t USBD_DeviceDesc_const[] __attribute__ ((aligned (4))) =
{
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    USB_WORD(0x0110),                       /*bcdUSB */
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    USB_OTG_MAX_EP0_SIZE,      /*bMaxPacketSize*/
    USB_WORD(USBD_VID),           /*idVendor*/
    USB_WORD(USBD_PID),           /*idProduct*/
    USB_WORD(0x0003),       // Device release number in BCD format
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,       /*Index of product string*/
    USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
    USBD_CFG_MAX_NUM            /*bNumConfigurations*/
}; /* USB_DeviceDescriptor */

static uint8_t USBD_DeviceDesc[sizeof(USBD_DeviceDesc_const)];

#define USB_SIZ_STRING_LANGID                   4
#define  USB_DESC_TYPE_STRING                              3
#define USBD_LANGID_STRING            0x0409

const uint8_t USBD_LangIDDesc_const[USB_SIZ_STRING_LANGID] __attribute__ ((aligned (4))) =
{
     USB_SIZ_STRING_LANGID,         
     USB_DESC_TYPE_STRING,       
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING), 
};

uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID];

/**
* @brief  USBD_USR_ManufacturerStrDescriptor 
*         return the manufacturer string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ManufacturerStrDescriptor( uint8_t speed , uint16_t *length)
{
  USBD_GetString ("McLionhead", USBD_StrDesc, length);
  return USBD_StrDesc;
}

#define USBD_PRODUCT_HS_STRING        "HS mode"
#define USBD_SERIALNUMBER_HS_STRING   "1"

#define USBD_PRODUCT_FS_STRING        "FS Mode"
#define USBD_SERIALNUMBER_FS_STRING   "1"

/**
* @brief  USBD_USR_ProductStrDescriptor 
*         return the product string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ProductStrDescriptor( uint8_t speed , uint16_t *length)
{
    USBD_GetString ("Laptop Keyboard", USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_SerialStrDescriptor 
*         return the serial number string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length)
{
    USBD_GetString (USBD_SERIALNUMBER_HS_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}


/**
* @brief  USBD_USR_ConfigStrDescriptor 
*         return the configuration string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length)
{
  USBD_GetString ("Config Desc", USBD_StrDesc, length);
  return USBD_StrDesc;  
}

/**
* @brief  USBD_USR_ConfigStrDescriptor 
*         return the configuration string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length)
{
  USBD_GetString ("Interface Desc", USBD_StrDesc, length);
  return USBD_StrDesc;  
}

/**
* @brief  USBD_USR_DeviceDescriptor 
*         return the device descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length)
{
  *length = sizeof(USBD_DeviceDesc_const);
  return (uint8_t*)USBD_DeviceDesc;
}


/**
* @brief  USBD_USR_LangIDStrDescriptor 
*         return the LangID string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length)
{
  *length =  sizeof(USBD_LangIDDesc);  
  return (uint8_t*)USBD_LangIDDesc;
}

const USBD_DEVICE USR_desc =
{
  USBD_USR_DeviceDescriptor,
  USBD_USR_LangIDStrDescriptor, 
  USBD_USR_ManufacturerStrDescriptor,
  USBD_USR_ProductStrDescriptor,
  USBD_USR_SerialStrDescriptor,
  USBD_USR_ConfigStrDescriptor,
  USBD_USR_InterfaceStrDescriptor,
  
};


/**
* @brief  USBD_USR_Init 
*         Displays the message on LCD for host lib initialization
* @param  None
* @retval None
*/
void USBD_USR_Init(void)
{   
	TRACE2 
}

/**
* @brief  USBD_USR_DeviceReset 
*         Displays the message on LCD on device Reset Event
* @param  speed : device speed
* @retval None
*/
void USBD_USR_DeviceReset(uint8_t speed )
{
	TRACE2 
}


/**
* @brief  USBD_USR_DeviceConfigured
*         Displays the message on LCD on device configuration Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceConfigured (void)
{
	TRACE2 
}

/**
* @brief  USBD_USR_DeviceSuspended 
*         Displays the message on LCD on device suspend Event
* @param  None
* @retval None
*/
void USBD_USR_DeviceSuspended(void)
{
//	TRACE2 
}

/**
* @brief  USBD_USR_DeviceResumed 
*         Displays the message on LCD on device resume Event
* @param  None
* @retval None
*/
void USBD_USR_DeviceResumed(void)
{
	TRACE2 
}

/**
* @brief  USBD_USR_DeviceConnected
*         Displays the message on LCD on device connection Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceConnected (void)
{
	TRACE2 
}

/**
* @brief  USBD_USR_DeviceDisonnected
*         Displays the message on LCD on device disconnection Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceDisconnected (void)
{
	TRACE2 
}

const USBD_Usr_cb_TypeDef USR_cb =
{
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,
  
  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,  
  
  
};

#define USBD_OK 0

/**
  * @brief  class_cb_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  class_cb_Init (void  *pdev, 
                               uint8_t cfgidx)
{
// open the endpoints
// CDC commands
    DCD_EP_Open(pdev,
                EP1_IN_ID,
                EP1_SIZE,
                USB_OTG_EP_INT);

    usb_start_receive();

  return USBD_OK;
}

/**
  * @brief  class_cb_Init
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  class_cb_DeInit (void  *pdev, 
                                 uint8_t cfgidx)
{
  /* Close HID EPs */
  DCD_EP_Close (pdev , EP1_IN_ID);
  DCD_EP_Close (pdev , EP1_OUT_ID);
  
  
  return USBD_OK;
}



// from Arduino keyboard library
// https://github.com/arduino-libraries/Keyboard/blob/master/src/Keyboard.cpp
// const uint8_t _hidReportDescriptor[] = 
// {
//     0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)  // 47
//     0x09, 0x06,                    // USAGE (Keyboard)
//     0xa1, 0x01,                    // COLLECTION (Application)
//     0x85, 0x02,                    //   REPORT_ID (2)
//     0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
// 
//     0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
//     0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
//     0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//     0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
//     0x75, 0x01,                    //   REPORT_SIZE (1)
// 
//     0x95, 0x08,                    //   REPORT_COUNT (8)
//     0x81, 0x02,                    //   INPUT (Data,Var,Abs)
//     0x95, 0x01,                    //   REPORT_COUNT (1)
//     0x75, 0x08,                    //   REPORT_SIZE (8)
//     0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
// 
//     0x95, 0x06,                    //   REPORT_COUNT (6)
//     0x75, 0x08,                    //   REPORT_SIZE (8)
//     0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//     0x25, 0x73,                    //   LOGICAL_MAXIMUM (115)
//     0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
// 
//     0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
//     0x29, 0x73,                    //   USAGE_MAXIMUM (Keyboard Application)
//     0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
//     0xc0,                          // END_COLLECTION
// // 47 bytes
// };

// wireshark capture
const uint8_t _hidReportDescriptor[] = 
{
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07, 
    0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01, 
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 
    0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01, 
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, 
    0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06, 
    0x75, 0x08, 0x15, 0x00, 0x26, 0xff, 0x00, 0x05, 
    0x07, 0x19, 0x00, 0x2a, 0xff, 0x00, 0x81, 0x00, 
    0x05, 0x0c, 0x09, 0x00, 0x15, 0x80, 0x25, 0x7f, 
    0x95, 0x40, 0x75, 0x08, 0xb1, 0x02, 0xc0
};

static uint8_t hidReportDescriptor[sizeof(_hidReportDescriptor)];

/**
  * @brief  class_cb_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
// Used for a keyboard or mouse.  Copy usbd_hid.c: USBD_HID_Setup
static uint8_t  class_cb_Setup (void  *pdev, 
                                USB_SETUP_REQ *req)
{
// TRACE
// print_number(req->wValue >> 8);
// print_lf();
    if((req->wValue >> 8) == USB_DESC_TYPE_HID_REPORT)
    {
        USBD_CtlSendData (pdev, hidReportDescriptor, sizeof(_hidReportDescriptor));

        initialized = 1;
        reset_uart();
    }

  return USBD_OK;
}

/**
  * @brief  class_cb_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  class_cb_DataIn (void  *pdev, 
                              uint8_t epnum)
{

  /* Ensure that the FIFO is empty before a new transfer, this condition could 
  be caused by  a new transfer before the end of the previous transfer */

  DCD_EP_Flush(pdev, EP1_IN_ID);


  return USBD_OK;
}

uint8_t class_cb_DataOut (void  *pdev, uint8_t epnum);


void usb_start_receive()
{
}

void usb_start_transmit(unsigned char *data, int len, int do_null)
{
    DCD_EP_Flush(&USB_OTG_dev,
        EP1_IN_ID);
	DCD_EP_Tx ( &USB_OTG_dev,
        EP1_IN_ID,
        data,
        len);

// sometimes it needs empty packets to avoid EOVERFLOW & sometimes it doesn't
// it might depend on the HID report descriptor
    if(do_null)
    {
        DCD_EP_Flush(&USB_OTG_dev,
            EP1_IN_ID);
    // send an empty packet to get the HID interrupt to not generate EOVERFLOW -75 errors
    // This is how it detects the end of the URB.
	    DCD_EP_Tx ( &USB_OTG_dev,
            EP1_IN_ID,
            data,
            0);
    }
}


#define USB_HID_CONFIG_DESC_SIZ       34
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define HID_DESCRIPTOR_TYPE           0x21
//#define HID_KEYBOARD_REPORT_DESC_SIZE    47
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

/* USB device Configuration Descriptor */
const uint8_t USBD_CfgDesc_const[] __attribute__ ((aligned (4))) =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ, /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xa0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  
  /************** Descriptor of keyboard interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x01,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of keyboard HID ********************/
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
//  47,/*wItemLength: Total length of Report descriptor*/
  79,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of keyboard endpoint ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/
  
  HID_EP | _EP_IN,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  HID_IN_PACKET, /*wMaxPacketSize: 8 bytes for keyboard */
  0x00,
  0x01,          /*bInterval: Polling Interval (10 ms)*/
  /* 34 bytes */
};




static uint8_t USBD_CfgDesc[sizeof(USBD_CfgDesc_const)];


/**
  * @brief  class_cb_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *class_cb_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (USBD_CfgDesc_const);
  return (uint8_t*)USBD_CfgDesc;
}


uint8_t class_cb_EP0_TxSent(void  *pdev)
{
	return 0;
}

uint8_t class_cb_EP0_RxReady(void  *pdev)
{
	return 0;
}

uint8_t  class_cb_sof (void *pdev)
{
}


static uint8_t  class_cb_OUT_Incplt (void  *pdev)
{
  return USBD_OK;
}


static uint8_t  class_cb_IN_Incplt (void  *pdev)
{
  return USBD_OK;
}




const USBD_Class_cb_TypeDef  USBD_class_cb = 
{
  class_cb_Init,   // Init
  class_cb_DeInit,  // DeInit
// control endpoint
  class_cb_Setup, // Setup
  class_cb_EP0_TxSent, /*EP0_TxSent*/  
  class_cb_EP0_RxReady, /*EP0_RxReady*/
// my endpoints
  class_cb_DataIn, /*DataIn*/
  class_cb_DataOut, /*DataOut*/
  class_cb_sof, /*SOF */
  class_cb_IN_Incplt,   // IsoINIncomplete
  class_cb_OUT_Incplt,    // IsoOUTIncomplete
  class_cb_GetCfgDesc,  // GetConfigDescriptor
#ifdef USB_OTG_HS_CORE  
  class_cb_GetCfgDesc, /* GetOtherConfigDescriptor */
#endif  
};

/* USB Standard Device Descriptor */
#define  USB_LEN_DEV_QUALIFIER_DESC                     0x0A
const uint8_t USBD_DeviceQualifierDesc_const[USB_LEN_DEV_QUALIFIER_DESC] __attribute__ ((aligned (4))) =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC];

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
 /* Note: On STM32F4-Discovery board only USB OTG FS core is supported. */
#ifdef USE_ACCURATE_TIME
	BSP_delay = 0;
#endif

  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure DM DP Pins */
// HS core
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
// HS core
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_OTG2_FS);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_OTG2_FS);

// The reference does this last
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_OTG_HS, ENABLE) ; 


  /* Intialize Timer for delay function */
  USB_OTG_BSP_TimeInit();   

}

void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
#ifdef ENABLE_INTERRUPT
  	NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable USB Interrupt */
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel = OTG_HS_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_Init(&NVIC_InitStructure);
#endif
}

void OTG_HS_IRQHandler(void)
{
//TRACE
    USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

#ifndef ENABLE_INTERRUPT
void handle_usb()
{
    USBD_OTG_ISR_Handler(&USB_OTG_dev);
}
#endif

void init_usb()
{
  	bzero((unsigned char*)&USB_OTG_dev, sizeof(USB_OTG_dev));

	memcpy(USBD_DeviceQualifierDesc, USBD_DeviceQualifierDesc_const, USB_LEN_DEV_QUALIFIER_DESC);
	memcpy(USBD_LangIDDesc, USBD_LangIDDesc_const, sizeof(USBD_LangIDDesc_const));
	memcpy(USBD_DeviceDesc, USBD_DeviceDesc_const, sizeof(USBD_DeviceDesc_const));
	memcpy(USBD_CfgDesc, USBD_CfgDesc_const, sizeof(USBD_CfgDesc_const));
    memcpy(hidReportDescriptor, _hidReportDescriptor, sizeof(_hidReportDescriptor));

 	USBD_Init(&USB_OTG_dev,
        USB_OTG_HS_CORE_ID,
        &USR_desc, 
        &USBD_class_cb, 
        &USR_cb);
}



