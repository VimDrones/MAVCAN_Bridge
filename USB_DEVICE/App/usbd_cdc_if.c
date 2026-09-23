/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include <mavlink.h>      // Add MAVLink header
#include "app_config.h"   // Include shared app config

// External references to global variables in main.c
extern volatile uint32_t mavlink_rx_count;
extern volatile uint8_t can_tx_count;
extern volatile uint8_t can_tx_write_idx;
extern volatile struct {
  uint32_t id;
  uint8_t is_extended;
  uint8_t is_rtr;
  uint8_t dlc;
  uint8_t data[8];
} can_tx_buffer[];

// USB ring buffer interface
#define USB_RX_PACKET_SIZE 64
#define USB_RX_RING_SIZE 8

typedef struct {
  uint8_t data[USB_RX_PACKET_SIZE];
  uint32_t len;
} usb_packet_t;

extern usb_packet_t usb_rx_ring[];
extern volatile uint8_t usb_rx_write_idx;
extern volatile uint8_t usb_rx_read_idx;
extern volatile uint8_t usb_rx_packets;

static volatile uint8_t usb_rx_paused = 0;
volatile uint32_t usb_rx_pause_count = 0;
volatile uint32_t usb_rx_resume_count = 0;
volatile uint16_t usb_control_line_state = 0;

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
// Store line coding for Windows compatibility
static USBD_CDC_LineCodingTypeDef LineCoding = {
  115200, /* baud rate*/
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* nb. of bits 8*/
};
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
static void CDC_ResetReceive_FS(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  usb_rx_write_idx = 0;
  usb_rx_read_idx = 0;
  usb_rx_packets = 0;
  usb_rx_paused = 0;
  usb_control_line_state = 0;
  __set_PRIMASK(primask);
}

/**
 * Resume a completed OUT transfer once the main loop has made room.
 * Leave an already armed transfer untouched, including during port setup.
 */
void CDC_ResumeReceive_FS(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (usb_rx_paused && usb_rx_packets < USB_RX_RING_SIZE &&
      hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
      hUsbDeviceFS.pClassData != NULL) {
    if (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK) {
      usb_rx_paused = 0;
      usb_rx_resume_count++;
    }
  }
  __set_PRIMASK(primask);
}

/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  CDC_ResetReceive_FS();
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  /* The CDC middleware prepares the initial OUT transfer after Init returns. */
  
  return USBD_OK;
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  CDC_ResetReceive_FS();
  return USBD_OK;
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  UNUSED(length);  // Add this line to suppress the warning
  
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      break;
    case CDC_GET_ENCAPSULATED_RESPONSE:
      break;
    case CDC_SET_COMM_FEATURE:
      break;
    case CDC_GET_COMM_FEATURE:
      break;
    case CDC_CLEAR_COMM_FEATURE:
      break;
    case CDC_SET_LINE_CODING:
      LineCoding.bitrate = ((USBD_CDC_LineCodingTypeDef*)pbuf)->bitrate;
      LineCoding.format = ((USBD_CDC_LineCodingTypeDef*)pbuf)->format;
      LineCoding.paritytype = ((USBD_CDC_LineCodingTypeDef*)pbuf)->paritytype;
      LineCoding.datatype = ((USBD_CDC_LineCodingTypeDef*)pbuf)->datatype;
      break;
    case CDC_GET_LINE_CODING:
      ((USBD_CDC_LineCodingTypeDef*)pbuf)->bitrate = LineCoding.bitrate;
      ((USBD_CDC_LineCodingTypeDef*)pbuf)->format = LineCoding.format;
      ((USBD_CDC_LineCodingTypeDef*)pbuf)->paritytype = LineCoding.paritytype;
      ((USBD_CDC_LineCodingTypeDef*)pbuf)->datatype = LineCoding.datatype;
      break;
    case CDC_SET_CONTROL_LINE_STATE:
      /* No-data CDC requests pass the complete setup request to Control. */
      usb_control_line_state = ((USBD_SetupReqTypedef*)pbuf)->wValue;
      /* Opening/closing the host port must not discard or rearm queued data. */
      break;
    case CDC_SEND_BREAK:
      break;
    default:
      break;
  }
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  // Add packet to ring buffer if space available
  if(*Len > 0 && *Len <= USB_RX_PACKET_SIZE && usb_rx_packets < USB_RX_RING_SIZE) {
    usb_packet_t *packet = (usb_packet_t*)&usb_rx_ring[usb_rx_write_idx];
    memcpy(packet->data, Buf, *Len);
    packet->len = *Len;
    
    usb_rx_write_idx = (usb_rx_write_idx + 1) % USB_RX_RING_SIZE;
    usb_rx_packets++;
  }

  /* This transfer is complete. Main-loop dequeue resumes it if we are full. */
  usb_rx_paused = 1;
  if (usb_rx_packets < USB_RX_RING_SIZE) {
    if (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK) {
      usb_rx_paused = 0;
    }
  } else {
    usb_rx_pause_count++;
  }
  
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  
  if (hcdc->TxState != 0) {
    return USBD_BUSY;
  }
  
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
