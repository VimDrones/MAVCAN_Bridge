/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  *
  * Copyright (c) 2025 Vimdrones.
  * All rights reserved.
  *
  * @author         : Huibean Luo
  * @email          : huibean.luo@vimdrones.com
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <mavlink.h>
#include "app_config.h" // Include the shared config file
#include "usbd_cdc_if.h" // Add this line for CDC functions
// #include <dronecan_msgs.h>
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define a short timeout for waiting on TX mailbox
#define CAN_TX_TIMEOUT_MS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
// MAVLink variables
uint8_t mav_system_id = 1;
uint8_t mav_component_id = 1;

// Heartbeat tracking
uint32_t last_heartbeat_time = 0;
uint32_t heartbeat_interval = 1000; // 1Hz heartbeat
uint32_t heartbeats_sent = 0;

// USB variables
uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
volatile uint32_t usb_rx_count = 0;
volatile uint8_t usb_rx_flag = 0;

volatile uint32_t usb_process_interval;
volatile uint32_t last_usb_process_time = 0; // Moved to global scope

// CAN frame structure
typedef struct {
  uint32_t id;        // CAN frame ID (standard or extended)
  uint8_t is_extended; // Flag for extended ID
  uint8_t is_rtr;     // Flag for remote transmission request
  uint8_t dlc;        // Data length code (0-8)
  uint8_t data[8];    // CAN frame data
} can_frame_t;

// Ring buffers for CAN frames
#define CAN_BUFFER_SIZE 128 
volatile can_frame_t can_rx_buffer[CAN_BUFFER_SIZE];
volatile uint8_t can_rx_write_idx = 0;
volatile uint8_t can_rx_read_idx = 0;
volatile uint8_t can_rx_count = 0;

volatile can_frame_t can_tx_buffer[CAN_BUFFER_SIZE];
volatile uint8_t can_tx_write_idx = 0;
volatile uint8_t can_tx_read_idx = 0;
volatile uint8_t can_tx_count = 0;

// Processing flags
volatile uint8_t usb_to_can_pending = 0;
volatile uint8_t can_to_usb_pending = 0;

// Status tracking
uint32_t rx_frames_count = 0;
uint32_t tx_frames_count = 0;
uint32_t mavlink_rx_count = 0;
uint32_t mavlink_tx_count = 0;
uint32_t buffer_overflow_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
uint32_t ProcessUSBtoCAN(void);
uint32_t ProcessCANtoUSB(void);
uint32_t SendCAN(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Pre-compute MAVLink CRC tables for better performance
static void mavlink_init_crc_tables(void) {
  // Initialize MAVLink CRC tables - forces table creation at startup instead of runtime
  uint16_t dummy_crc = crc_calculate((uint8_t*)"test", 4);
  (void)dummy_crc; // Prevent unused variable warning
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // Precompute MAVLink CRC tables
  mavlink_init_crc_tables();
  
  // Rest of initialization...
  // Enable CAN interrupts
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }

  // Activate CAN RX notification
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }

  // Initialize ring buffers
  can_rx_write_idx = 0;
  can_rx_read_idx = 0;
  can_tx_write_idx = 0;
  can_tx_read_idx = 0;

  // Make sure USB is ready to receive data using interrupts
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Get current time once per loop
    uint32_t current_time = HAL_GetTick();

    static uint32_t last_usb_poll_time = 0;
    if (current_time - last_usb_poll_time >= 1) { // Poll every 1ms
      last_usb_poll_time = current_time;
      // Always keep USB endpoint ready
      if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && !usb_rx_flag) {
          USBD_CDC_ReceivePacket(&hUsbDeviceFS);
      }
    }

    // Process USB data when available
    if (usb_rx_flag) {
        ProcessUSBtoCAN();
        usb_rx_flag = 0;
        
        // Send frames immediately after processing
        if (can_tx_count > 0) {
            SendCAN(); // Send one frame immediately
        }
    }

    // Process CAN TX queue - up to 8 frames at once
    if (can_tx_count > 0) {
        for (int i = 0; i < 8 && can_tx_count > 0; i++) {
            uint32_t sent = SendCAN();
            if (sent == 0) break;
        }
    }
    
    // Send MAVLink heartbeat at 1Hz
    if (current_time - last_heartbeat_time >= heartbeat_interval) {
      SendMavlinkHeartbeat();
      last_heartbeat_time = current_time;
    }
    
    // Process CAN RX with lowest priority
    if (can_rx_count > 0) {
      ProcessCANtoUSB();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  // Configure CAN filter to accept all messages
  CAN_FilterTypeDef canFilterConfig;
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0;
  canFilterConfig.FilterIdLow = 0;
  canFilterConfig.FilterMaskIdHigh = 0;
  canFilterConfig.FilterMaskIdLow = 0;
  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  
  if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_TX_Pin|LED_RX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_TX_Pin LED_RX_Pin */
  GPIO_InitStruct.Pin = LED_TX_Pin|LED_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief CAN RX FIFO 0 callback
 * @param hcan: CAN handle pointer
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  
  // Get the message from CAN FIFO
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    // Check if there's space in the buffer
    if (can_rx_count < CAN_BUFFER_SIZE) {
      // Store the frame in the circular buffer
      can_frame_t *frame = (can_frame_t *)&can_rx_buffer[can_rx_write_idx];
      
      // Extract ID information
      if (rxHeader.IDE == CAN_ID_STD) {
        frame->id = rxHeader.StdId;
        frame->is_extended = 0;
      } else {
        frame->id = rxHeader.ExtId;
        frame->is_extended = 1;
      }
      
      // Extract RTR flag and data
      frame->is_rtr = (rxHeader.RTR == CAN_RTR_REMOTE) ? 1 : 0;
      frame->dlc = rxHeader.DLC;
      
      if (rxHeader.RTR != CAN_RTR_REMOTE) {
        memcpy((void*)frame->data, rxData, frame->dlc);
      }
      
      // Update indexes and counter with atomic operations
      __disable_irq();
      can_rx_write_idx = (can_rx_write_idx + 1) % CAN_BUFFER_SIZE;
      can_rx_count++;
      can_to_usb_pending = 1;
      __enable_irq();
      
      // Toggle LED to indicate reception
      HAL_GPIO_TogglePin(GPIOA, LED_RX_Pin);
      rx_frames_count++;
    } else {
      // Buffer is full
      buffer_overflow_count++;
    }
  }
}

/**
 * @brief Process USB to CAN data safely
 * @retval Number of processed bytes
 */
uint32_t ProcessUSBtoCAN(void)
{
  static mavlink_message_t msg;
  static mavlink_status_t status;
  uint32_t processed = 0;
  
  // Skip flag check - we already checked in the caller
  uint32_t bytes = usb_rx_count;
  
  // Process bytes in larger chunks for better cache efficiency
  for (uint32_t i = 0; i < bytes; ) {
    // Process up to 16 bytes at once
    uint32_t chunk_end = i + 16;
    if (chunk_end > bytes) chunk_end = bytes;
    
    for (; i < chunk_end; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, usb_rx_buffer[i], &msg, &status)) {
        mavlink_rx_count++;
        
        if (msg.msgid == MAVLINK_MSG_ID_CAN_FRAME) {
          // Extract directly to minimize copies
          mavlink_can_frame_t can_frame;
          mavlink_msg_can_frame_decode(&msg, &can_frame);
          
          if (can_tx_count < CAN_BUFFER_SIZE) {
            uint8_t idx = can_tx_write_idx;
            volatile can_frame_t *frame = &can_tx_buffer[idx];
            
            // Fast extractions
            frame->id = can_frame.id & 0x1FFFFFFF;
            frame->is_extended = (can_frame.id & 0x80000000) ? 1 : 0;
            frame->is_rtr = (can_frame.id & 0x40000000) ? 1 : 0;
            frame->dlc = can_frame.len <= 8 ? can_frame.len : 8;
            
            if (!frame->is_rtr && frame->dlc > 0) {
              // Direct memcpy to avoid for loop
              memcpy((void*)frame->data, can_frame.data, frame->dlc);
            }
            
            // Minimal critical section
            __disable_irq();
            can_tx_write_idx = (idx + 1) % CAN_BUFFER_SIZE;
            can_tx_count++;
            __enable_irq();
          }
        }
      }
    }
  }
  
  processed = bytes;
  return processed;
}

/**
 * @brief Process CAN to USB data
 * @retval Number of frames processed
 */
uint32_t ProcessCANtoUSB(void)
{
  uint32_t processed = 0;
  mavlink_message_t msg;
  uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN];
  
  // Process all available CAN frames
  while (can_rx_count > 0) {
    can_frame_t frame;
    
    // Get the frame from the buffer atomically
    __disable_irq();
    memcpy(&frame, (void*)&can_rx_buffer[can_rx_read_idx], sizeof(can_frame_t));
    can_rx_read_idx = (can_rx_read_idx + 1) % CAN_BUFFER_SIZE;
    can_rx_count--;
    __enable_irq();
    
    // Create MAVLink CAN_FRAME message
    uint32_t frame_id = frame.id;

    // Add extended ID flag if needed
    if (frame.is_extended) {
      frame_id |= 0x80000000;
    }

    // Add RTR flag if needed
    if (frame.is_rtr) {
      frame_id |= 0x40000000;
    }

    // Pack the MAVLink message
    mavlink_msg_can_frame_pack(
      mav_system_id,
      mav_component_id,
      &msg,
      0,
      0,
      0,
      frame.dlc,
      frame_id,
      frame.data
    );
                           
    // Get the buffer to send
    uint16_t mavlink_len = mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
    
    // Send over USB
    if (USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state) {
      if (CDC_Transmit_FS(mavlink_buffer, mavlink_len) == USBD_OK) {
        mavlink_tx_count++;
      }
    }
    
    processed++;
  }
  
  return processed;
}

/**
 * @brief Hyper-optimized CAN transmission function for 400+ FPS
 * @retval Number of frames sent
 */
uint32_t SendCAN(void)
{
  uint32_t sent = 0;
  
  // Maximum burst for high throughput
  for (uint32_t attempt = 0; attempt < 32 && can_tx_count > 0; attempt++) {
    // Fast register check for available mailboxes
    uint32_t tsr = hcan.Instance->TSR;
    
    // No mailboxes available - exit immediately
    if ((tsr & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0) {
      break;
    }
    
    // Select mailbox with lowest index (highest priority)
    uint8_t mailbox = 
      (tsr & CAN_TSR_TME0) ? 0 : 
      (tsr & CAN_TSR_TME1) ? 1 : 2;
    
    // Extract frame atomically with minimal critical section
    can_frame_t frame;
    __disable_irq();
    memcpy(&frame, (void*)&can_tx_buffer[can_tx_read_idx], sizeof(can_frame_t));
    can_tx_read_idx = (can_tx_read_idx + 1) % CAN_BUFFER_SIZE;
    can_tx_count--;
    __enable_irq();
    
    // Prepare TIR register value outside the register write
    uint32_t tir = 0;
    if (frame.is_extended) {
      tir = ((frame.id << 3) | CAN_TI0R_IDE);
    } else {
      tir = (frame.id << 21);
    }
    if (frame.is_rtr) {
      tir |= CAN_TI0R_RTR;
    }
    
    // Write registers in one shot each
    hcan.Instance->sTxMailBox[mailbox].TDTR = frame.dlc;
    hcan.Instance->sTxMailBox[mailbox].TDLR = 
      ((uint32_t)frame.data[3] << 24) |
      ((uint32_t)frame.data[2] << 16) |
      ((uint32_t)frame.data[1] << 8) |
      ((uint32_t)frame.data[0]);
    hcan.Instance->sTxMailBox[mailbox].TDHR = 
      ((uint32_t)frame.data[7] << 24) |
      ((uint32_t)frame.data[6] << 16) |
      ((uint32_t)frame.data[5] << 8) |
      ((uint32_t)frame.data[4]);
    
    // Write TIR last with TXRQ bit to trigger transmission
    hcan.Instance->sTxMailBox[mailbox].TIR = tir | CAN_TI0R_TXRQ;
    
    sent++;
    tx_frames_count++;
    HAL_GPIO_TogglePin(GPIOA, LED_TX_Pin);
  }
  
  return sent;
}

/**
 * @brief CAN Error callback for high-speed operation
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t error = HAL_CAN_GetError(hcan);
  
  // Only reset on bus-off error to avoid losing time on minor errors
  if (error & HAL_CAN_ERROR_BOF) {
    HAL_CAN_ResetError(hcan);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
}

/**
 * @brief Send MAVLink heartbeat message
 */
void SendMavlinkHeartbeat(void)
{
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  
  // Pack heartbeat message
  // Parameters: system_id, component_id, msg, type, autopilot, base_mode, custom_mode, system_status
  mavlink_msg_heartbeat_pack(
    mav_system_id, 
    mav_component_id,
    &msg,
    MAV_TYPE_ONBOARD_CONTROLLER, // Type of the component
    MAV_AUTOPILOT_INVALID,       // Autopilot type (not applicable)
    0,                           // System mode
    0,                           // Custom mode
    MAV_STATE_ACTIVE             // System state (active)
  );
  
  // Convert message to buffer
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  
  // Send message if USB is ready
  if (USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state) {
    if (CDC_Transmit_FS(buffer, len) == USBD_OK) {
      heartbeats_sent++;
      mavlink_tx_count++;  // Count as normal MAVLink TX
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
