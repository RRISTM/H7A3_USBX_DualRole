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



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_QUEUE_SIZE                               1
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

extern HCD_HandleTypeDef                 hhcd_USB_OTG_HS;
TX_THREAD                                keyboard_app_thread;
TX_THREAD                                mouse_app_thread;
TX_QUEUE                                 ux_app_MsgQueue;
UX_HOST_CLASS_HID                        *hid;
UX_HOST_CLASS_HID_CLIENT                 *hid_client;
UX_HOST_CLASS_HID_MOUSE                  *mouse;
UX_HOST_CLASS_HID_KEYBOARD               *keyboard;

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
__ALIGN_BEGIN ux_app_devInfotypeDef       ux_dev_info  __ALIGN_END;

CHAR *pointer_mouse_app_thread;
CHAR *pointer_APP_QUEUE_SIZE;
/* CDC Class Calling Parameter structure */
UX_SLAVE_CLASS_CDC_ACM_PARAMETER    cdc_acm_parameter;

/* Define constants.  */
#define USBX_APP_STACK_SIZE       1024
#define USBX_MEMORY_SIZE          (16 * 1024)

UCHAR *pointer_ux;
UCHAR *pointer_ux_app_thread;
UCHAR *pointer_ux_cdc_read_thread;
 uint32_t connectionState = 0;


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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void  usbx_app_thread_entry(ULONG arg);
void usbx_app_start_device(void);
void usbx_app_stop_device(void);
void usbx_app_start_host(void);
void usbx_app_stop_host(void);

void usbx_app_process_host(void);

UINT  ux_host_event_callback(ULONG event, UX_HOST_CLASS *p_host_class, VOID *p_instance);
VOID  ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code);

static void  USBH_DriverVBUS(uint8_t state);
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


    /* Get_Device_Framework_Full_Speed and get the length */
    device_framework_full_speed = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED,
                                  &device_framework_fs_length);

    /* Get_String_Framework and get the length */
    string_framework = USBD_Get_String_Framework(&string_framework_length);

    /* Get_Language_Id_Framework and get the length */
    language_id_framework = USBD_Get_Language_Id_Framework(&languge_id_framework_length);

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

    /*host*/

    /* Allocate the stack for thread 1. */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer_mouse_app_thread,
                       USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }



  /* Allocate Memory for the Queue */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer_APP_QUEUE_SIZE,
                       APP_QUEUE_SIZE * sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  /* USER CODE END MX_USBX_Host_Init */

  return ret;
}

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern HCD_HandleTypeDef hhcd_USB_OTG_HS;
uint32_t usbDeviceHostState=3;


void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  if((usbDeviceHostState == 0) || (usbDeviceHostState==1) ||  (usbDeviceHostState==2)){

    HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  }else{
    HAL_HCD_IRQHandler(&hhcd_USB_OTG_HS);
  }
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void usbx_app_thread_entry(ULONG arg)
{
  GPIO_PinState recognizeState=0; 

  uint32_t oldUsbDeviceHostState=0x2;
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
    // usbx_app_start_device();
  /* register a callback error function */
  _ux_utility_error_callback_register(&ux_host_error_callback);
  while(1){


    recognizeState = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10);
    // if((recognizeState!=oldRecognizeState)||(usbDeviceHostState!=oldUsbDeviceHostState))
    // {
      switch (usbDeviceHostState)
      {
        case 0:
          usbx_app_start_device();
          usbDeviceHostState=1;
        break;
        case 1:
          if(recognizeState == GPIO_PIN_RESET){
            usbDeviceHostState=2;
          }
        break;
        case 2:
          usbx_app_stop_device();
          usbDeviceHostState=3;
        break;
        case 3:
          usbx_app_start_host();
          usbDeviceHostState=4;
        break;
        case 4:
          usbx_app_process_host();
          if(recognizeState==GPIO_PIN_SET){
            usbDeviceHostState=5;
          }
        break;

        case 5:
          usbx_app_stop_host();
          usbDeviceHostState=0;
        break;

        default:
        break;
      }
      tx_thread_sleep(MS_TO_TICK(10)); tx_thread_sleep(MS_TO_TICK(10));
  }
}


void usbx_app_start_device(){
  /* USER CODE BEGIN USB_Device_Init_PreTreatment_0 */

  /* Initialize USBX Memory */

  UINT retVal;



  /* The code below is required for installing the device portion of USBX.
    In this application */
  retVal =ux_device_stack_initialize(NULL,
                                0U,
                                device_framework_full_speed,
                                device_framework_fs_length,
                                string_framework,
                                string_framework_length,
                                language_id_framework,
                                languge_id_framework_length,
                NULL);
  if(retVal != TX_SUCCESS){
    while(1);
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
	    while(1);
  }

  /* Create the usbx_cdc_acm_read_thread_entry thread. */
  if (tx_thread_create(&ux_cdc_read_thread, "cdc_acm_read_usbx_app_thread_entry",
                       usbx_cdc_acm_read_thread_entry, 1, pointer_ux_cdc_read_thread,
                       USBX_APP_STACK_SIZE, 20, 20, TX_NO_TIME_SLICE,
                       TX_AUTO_START) != TX_SUCCESS)
  {
	    while(1);
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

  retVal = tx_thread_terminate(&ux_cdc_read_thread);
  if(retVal != TX_SUCCESS){
    while(1);
  }
  retVal = tx_thread_delete(&ux_cdc_read_thread);
  if(retVal != TX_SUCCESS){
    while(1);
  }
  retVal =_ux_dcd_stm32_uninitialize((ULONG)USB_OTG_HS, (ULONG)&hpcd_USB_OTG_HS);
  if(retVal != TX_SUCCESS){
    while(1);
  }
  retVal = ux_device_stack_uninitialize();
  if(retVal != TX_SUCCESS){
    while(1);
  }

  MX_USB_OTG_HS_PCD_DeInit();
}

void usbx_app_start_host(){
        /* The code below is required for installing the host portion of USBX. */
        if (ux_host_stack_initialize(ux_host_event_callback) != UX_SUCCESS)
        {
          return UX_ERROR;
        }

        /* Register hid class. */
        if (ux_host_stack_class_register(_ux_system_host_class_hid_name,
                                        _ux_host_class_hid_entry) != UX_SUCCESS)
        {
          return UX_ERROR;
        }

        /* Register HID Mouse client */
        if (ux_host_class_hid_client_register(_ux_system_host_class_hid_client_mouse_name,
                                              ux_host_class_hid_mouse_entry) != UX_SUCCESS)
        {
          return UX_ERROR;
        }

        /* Initialize the LL driver */
        MX_USB_OTG_HS_HCD_Init();

        /* Register all the USB host controllers available in this system.  */
        if (ux_host_stack_hcd_register(_ux_system_host_hcd_stm32_name,
                                      _ux_hcd_stm32_initialize, USB_OTG_HS_PERIPH_BASE,
                                      (ULONG)&hhcd_USB_OTG_HS) != UX_SUCCESS)
        {
          return UX_ERROR;
        }

        /* Drive vbus */
        USBH_DriverVBUS(USB_VBUS_TRUE);

        /* Enable USB Global Interrupt*/
        HAL_HCD_Start(&hhcd_USB_OTG_HS);

              /* Create the HID mouse App thread. */
        if (tx_thread_create(&mouse_app_thread, "thread 1", hid_mouse_thread_entry, 0,
                            pointer_mouse_app_thread, USBX_APP_STACK_SIZE, 30, 30, 1,
                            TX_AUTO_START) != TX_SUCCESS)
        {
          return TX_THREAD_ERROR;
        }
        /* Create the MsgQueue */
        if (tx_queue_create(&ux_app_MsgQueue, "Message Queue app", TX_1_ULONG,
                            pointer_APP_QUEUE_SIZE, APP_QUEUE_SIZE * sizeof(ULONG)) != TX_SUCCESS)
        {
          return TX_QUEUE_ERROR;
        }
}

void usbx_app_process_host(){
  
          /* Wait for a hid device to be connected */
          if (tx_queue_receive(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT)== TX_SUCCESS)
          {
            if (ux_dev_info.Dev_state == Device_connected)
            {
              switch (ux_dev_info.Device_Type)
              {
                case Mouse_Device :
                  mouse = hid_client-> ux_host_class_hid_client_local_instance;
                  break;

                case Keyboard_Device :
                  keyboard = hid_client-> ux_host_class_hid_client_local_instance;

                  break;

                case Unknown_Device :
                  break;

                default :
                  break;
              }
            }
            else
            {
              /* clear hid_client local instance */
              mouse = NULL;
              keyboard = NULL;
            }
          }

}

void usbx_app_stop_host(){
 UINT retVal;
  /* Enable USB Global Interrupt*/
  HAL_HCD_Stop(&hhcd_USB_OTG_HS);

        /* Drive vbus */
  USBH_DriverVBUS(USB_VBUS_FALSE);


  // __disable_irq();

    /* Register all the USB host controllers available in this system.  */
  retVal =ux_host_stack_hcd_unregister(_ux_system_host_hcd_stm32_name,
                                  USB_OTG_HS_PERIPH_BASE,
                                (ULONG)&hhcd_USB_OTG_HS);
  if (retVal != TX_SUCCESS){
    while(1);
  }

        /* Register hid class. */
  retVal = ux_host_stack_class_unregister(_ux_host_class_hid_entry);
  if (retVal != TX_SUCCESS){
    while(1);
  }
  retVal = ux_host_stack_uninitialize();
  if (retVal != TX_SUCCESS){
    while(1);
  }
  retVal =tx_thread_terminate(&mouse_app_thread);
  if (retVal != TX_SUCCESS){
    while(1);
  }

  retVal = tx_thread_delete(&mouse_app_thread);
  if (retVal != TX_SUCCESS){
    while(1);
  }
  retVal = tx_queue_delete(&ux_app_MsgQueue);
  if (retVal != TX_SUCCESS){
    while(1);
  }
  // __enable_irq();
  MX_USB_OTG_HS_HCD_DeInit();
}

/**
* @brief  Drive VBUS.
* @param  state : VBUS state
*          This parameter can be one of the these values:
*           1 : VBUS Active
*           0 : VBUS Inactive
* @retval Status
*/
static void USBH_DriverVBUS(uint8_t state)
{
  /* USER CODE BEGIN 0 */

  /* USER CODE END 0*/

  if (state == USB_VBUS_TRUE)
  {
    /* Drive high Charge pump */
    /* Add IOE driver control */
    /* USER CODE BEGIN DRIVE_HIGH_CHARGE_FOR_HS */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    /* USER CODE END DRIVE_HIGH_CHARGE_FOR_HS */
  }
  else
  {
    /* Drive low Charge pump */
    /* Add IOE driver control */
    /* USER CODE BEGIN DRIVE_LOW_CHARGE_FOR_HS */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    /* USER CODE END DRIVE_LOW_CHARGE_FOR_HS */
  }

  HAL_Delay(200);
}

/**
* @brief ux_host_event_callback
* @param ULONG event
           This parameter can be one of the these values:
             1 : UX_DEVICE_INSERTION
             2 : UX_DEVICE_REMOVAL
             3 : UX_HID_CLIENT_INSERTION
             4 : UX_HID_CLIENT_REMOVAL
         UX_HOST_CLASS * Current_class
         VOID * Current_instance
* @retval Status
*/
UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *Current_class, VOID *Current_instance)
{
  UINT status;
  UX_HOST_CLASS *hid_class;

  switch (event)
  {
    case UX_DEVICE_INSERTION :
      /* Get current Hid Class */
      status = ux_host_stack_class_get(_ux_system_host_class_hid_name, &hid_class);

      if (status == UX_SUCCESS)
      {
        if ((hid_class == Current_class) && (hid == NULL))
        {
          /* Get current Hid Instance */
          hid = Current_instance;
          /* Get the HID Client */
          hid_client = hid ->ux_host_class_hid_client;

          if (hid->ux_host_class_hid_client->ux_host_class_hid_client_status != (ULONG) UX_HOST_CLASS_INSTANCE_LIVE)
          {
            ux_dev_info.Device_Type = Unknown_Device;
          }
          /* Check the HID_client if this is a HID mouse device. */
          if (ux_utility_memory_compare(hid_client -> ux_host_class_hid_client_name,
                                        _ux_system_host_class_hid_client_mouse_name,
                                        ux_utility_string_length_get(_ux_system_host_class_hid_client_mouse_name)) == UX_SUCCESS)
          {
            /* update HID device Type */
            ux_dev_info.Device_Type = Mouse_Device;

            /* put a message queue to usbx_app_thread_entry */
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }

          /* Check the HID_client if this is a HID keyboard device. */
          else if (ux_utility_memory_compare(hid_client -> ux_host_class_hid_client_name,
                                             _ux_system_host_class_hid_client_keyboard_name,
                                             ux_utility_string_length_get(_ux_system_host_class_hid_client_keyboard_name)) == UX_SUCCESS)
          {
            /* update HID device Type */
            ux_dev_info.Device_Type = Keyboard_Device;

            /* put a message queue to usbx_app_thread_entry */
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }
          else
          {
            ux_dev_info.Device_Type = Unknown_Device;
            ux_dev_info.Dev_state = Device_connected;
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }
        }
      }
      else
      {
      }
      break;

    case UX_DEVICE_REMOVAL :

      if (Current_instance == hid)
      {
        /* Free Instance */
        hid = NULL;
        ux_dev_info.Dev_state   = No_Device;
        ux_dev_info.Device_Type = Unknown_Device;
      }
      break;

    case UX_HID_CLIENT_INSERTION :
      ux_dev_info.Dev_state = Device_connected;
      break;

    case UX_HID_CLIENT_REMOVAL:
      ux_dev_info.Dev_state   =  Device_disconnected;
      ux_dev_info.Device_Type =  Unknown_Device;
      tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);

      break;

    default:
      break;

  }

  return (UINT) UX_SUCCESS;
}

/**
* @brief ux_host_error_callback
* @param ULONG event
         UINT system_context
         UINT error_code
* @retval Status
*/
VOID ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code)
{
  switch (error_code)
  {
    case UX_DEVICE_ENUMERATION_FAILURE :

      ux_dev_info.Device_Type = Unknown_Device;
      ux_dev_info.Dev_state   = Device_connected;
      tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
      break;

    case  UX_NO_DEVICE_CONNECTED :
      break;

    default:
      break;
  }
}
/* USER CODE END 1 */
