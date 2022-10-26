/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_host.c
  * @author  MCD Application Team
  * @brief   USBX host applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_usbx_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_otg.h"
#include "ux_system.h"
#include "ux_utility.h"
#include "ux_device_stack.h"
#include "ux_dcd_stm32.h"
#include "ux_device_descriptors.h"
#include "ux_device_cdc_acm.h"
#include "app_azure_rtos_config.h"
#include "app_usbx_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD               ux_app_thread;
TX_THREAD               ux_cdc_read_thread;
TX_THREAD               ux_cdc_write_thread;

TX_EVENT_FLAGS_GROUP    EventFlag;

/* CDC Class Calling Parameter structure */
UX_SLAVE_CLASS_CDC_ACM_PARAMETER    cdc_acm_parameter;

/* Define constants.  */
#define USBX_APP_STACK_SIZE       1024
#define USBX_MEMORY_SIZE          (16 * 1024)

UCHAR *pointer_ux;
UCHAR *pointer_ux_app_thread;
UCHAR *pointer_ux_cdc_read_thread;
 uint32_t connectionState = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void  usbx_app_thread_entry(ULONG arg);
void usbx_app_start_device(void);
void usbx_app_stop_device(void);
void usbx_app_start_host(void);
void usbx_app_stop_host(void);
/* USER CODE END PFP */
/**
  * @brief  Application USBX Host Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_USBX_Host_Init(VOID *memory_ptr)
{
  UINT ret = UX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_USBX_Host_MEM_POOL */
    (void)byte_pool;
  /* USER CODE END MX_USBX_Host_MEM_POOL */

  /* USER CODE BEGIN MX_USBX_Host_Init */

    UCHAR *pointer;


    /* Allocate USBX_MEMORY_SIZE. */
    if (tx_byte_allocate(byte_pool, (VOID **) &pointer_ux,
                         USBX_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
      return TX_POOL_ERROR;
    }



    /* Allocate the stack for main_usbx_app_thread_entry. */
    if (tx_byte_allocate(byte_pool, (VOID **) &pointer_ux_app_thread,
                         USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
      return TX_POOL_ERROR;
    }

    /* Create the usbx_app_thread_entry main thread. */
    if (tx_thread_create(&ux_app_thread, "main_usbx_app_thread_entry",
                         usbx_app_thread_entry, 0, pointer_ux_app_thread, USBX_APP_STACK_SIZE,
                         20, 20, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
      return TX_THREAD_ERROR;
    }

    /* Allocate the stack for usbx_cdc_acm_read_thread_entry. */
    if (tx_byte_allocate(byte_pool, (VOID **) &pointer_ux_cdc_read_thread,
                         USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
      return TX_POOL_ERROR;
    }
  /* USER CODE END MX_USBX_Host_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
void usbx_app_thread_entry(ULONG arg)
{
  GPIO_PinState recognizeState=0; 
  uint32_t usbDeviceHostState=0;
  uint32_t oldUsbDeviceHostState=0xFF;
  GPIO_PinState oldRecognizeState=0; 
  /**
   * 0 - nothign connected - host active
   * 1 - device found
   * 2 - host connected
   * 3 - host connected device active
   * 4 - host disconnected device active
   */
  if (ux_system_initialize(pointer_ux, USBX_MEMORY_SIZE, UX_NULL, 0) != UX_SUCCESS)
  {
    return UX_ERROR;
  }
    //usbx_app_start_device();
  while(1){


    recognizeState = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10);
    if((recognizeState!=oldRecognizeState)||(usbDeviceHostState!=oldUsbDeviceHostState))
    {
      switch (usbDeviceHostState)
      {
      case 0:
        if(recognizeState == GPIO_PIN_SET){
          usbDeviceHostState=1;
          usbx_app_start_device();

        }
        break;
        case 1:
        if(recognizeState == GPIO_PIN_RESET){
          usbDeviceHostState=0;
          usbx_app_stop_device();
        }
        break;

      default:

        break;
      }
      oldRecognizeState=recognizeState;
      usbDeviceHostState=oldUsbDeviceHostState;
    }
  }



}


void usbx_app_start_device(){
  /* USER CODE BEGIN USB_Device_Init_PreTreatment_0 */
    /* Device framework FS length*/
  ULONG device_framework_fs_length;
  /* Device String framework length*/
  ULONG string_framework_length;
  /* Device language id framework length*/
  ULONG languge_id_framework_length;
  /* Device Framework Full Speed */
  UCHAR *device_framework_full_speed;
  /* String Framework*/
  UCHAR *string_framework;
  /* Language_Id_Framework*/
  UCHAR *language_id_framework;
  /* Initialize USBX Memory */

  /* Get_Device_Framework_Full_Speed and get the length */
  device_framework_full_speed = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED,
                                &device_framework_fs_length);

  /* Get_String_Framework and get the length */
  string_framework = USBD_Get_String_Framework(&string_framework_length);

  /* Get_Language_Id_Framework and get the length */
  language_id_framework = USBD_Get_Language_Id_Framework(&languge_id_framework_length);

  /* The code below is required for installing the device portion of USBX.
    In this application */
  if (ux_device_stack_initialize(NULL,
                                0U,
                                device_framework_full_speed,
                                device_framework_fs_length,
                                string_framework,
                                string_framework_length,
                                language_id_framework,
                                languge_id_framework_length,
                NULL) != UX_SUCCESS)
  {
    return UX_ERROR;
  }

  /* Initialize the cdc class parameters for the device. */
  cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate = CDC_Init_FS;

  /* Deinitialize the cdc class parameters for the device. */
  cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = CDC_DeInit_FS;

  /* Manage the CDC class requests */
  cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change = ux_app_parameters_change;

  /* Registers a slave class to the slave stack. The class is connected with
    interface 0 */
  if (ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
                                    ux_device_class_cdc_acm_entry, 1, 0,
                                    (VOID *)&cdc_acm_parameter) != UX_SUCCESS)
  {
    return UX_ERROR;
  }

  /* Create the usbx_cdc_acm_read_thread_entry thread. */
  if (tx_thread_create(&ux_cdc_read_thread, "cdc_acm_read_usbx_app_thread_entry",
                       usbx_cdc_acm_read_thread_entry, 1, pointer_ux_cdc_read_thread,
                       USBX_APP_STACK_SIZE, 20, 20, TX_NO_TIME_SLICE,
                       TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }
  /* USER CODE END USB_Device_Init_PreTreatment_0 */

  /* Enable the USB voltage level detector */
  HAL_PWREx_EnableUSBVoltageDetector();

  /* USB_OTG_HS init function */
  MX_USB_OTG_HS_PCD_Init();

  /* USER CODE BEGIN USB_Device_Init_PreTreatment_1 */

  /* Set Rx FIFO */
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_HS, 0x200);

  /* Set Tx FIFO 0 */
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 0, 0x10);

  /* Set Tx FIFO 2 */
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 1, 0x10);

  /* Set Tx FIFO 3 */
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 2, 0x20);

  /* USER CODE END USB_Device_Init_PreTreatment_1 */

  /* initialize the device controller driver */
  ux_dcd_stm32_initialize((ULONG)USB_OTG_HS, (ULONG)&hpcd_USB_OTG_HS);

  /* Start the USB device */
  HAL_PCD_Start(&hpcd_USB_OTG_HS);

}

void usbx_app_stop_device(){
  UINT retVal;
  HAL_PCD_Stop(&hpcd_USB_OTG_HS);

  retVal = ux_device_stack_uninitialize();
  if(TX_SUCCESS != TX_SUCCESS){
    while(1);
  }
  retVal = tx_thread_terminate(&ux_cdc_read_thread);
  if(TX_SUCCESS != TX_SUCCESS){
    while(1);
  }
  retVal = tx_thread_delete(&ux_cdc_read_thread);
  if(TX_SUCCESS != TX_SUCCESS){
    while(1);
  }
  retVal = ux_device_stack_uninitialize();
  if(TX_SUCCESS != TX_SUCCESS){
    while(1);
  }

  retVal =_ux_dcd_stm32_uninitialize((ULONG)USB_OTG_HS, (ULONG)&hpcd_USB_OTG_HS);
  if(TX_SUCCESS != TX_SUCCESS){
    while(1);
  }
  MX_USB_OTG_HS_PCD_DeInit();
}

void usbx_app_start_host(){

}

void usbx_app_stop_host(){

}
/* USER CODE END 1 */
