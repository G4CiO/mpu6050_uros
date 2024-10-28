/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rcl/init_options.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>
#include <mpu6050.h>
#include <math.h>
#include <geometry_msgs/msg/twist.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define G2M_S2 9.81
#define DEG2RAD M_PI / 180.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;

rcl_timer_t mpu6050_timer;
rclc_executor_t executor;
rcl_publisher_t mpu6050_publisher;
sensor_msgs__msg__Imu mpu6050_msg;

MPU6050_t MPU6050;

rcl_publisher_t cmd_vel_publisher;
geometry_msgs__msg__Twist pub_msg;

typedef struct {
    double linearVelocityX;   // Linear velocity in x-axis (m/s)
    double angularVelocityZ;  // Angular velocity in z-axis (rad/s)
} VelocityOutput;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
VelocityOutput convertRollPitchToVelocity(double roll, double pitch) {
    VelocityOutput output;

    // Convert roll and pitch from degrees to radians
    double rollRad = roll * DEG2RAD;
    double pitchRad = pitch * DEG2RAD;

    // Define a scaling factor for the conversion (tune this as needed)
    double linearScale = 5.0;  // Adjust based on desired speed and response
    double angularScale = 3.0;

    // Calculate linear velocity in x-axis (m/s) from roll
    output.linearVelocityX = linearScale * cos(pitchRad) * sin(rollRad);

    // Calculate angular velocity in z-axis (rad/s) from pitch
    output.angularVelocityZ = angularScale * sin(pitchRad);

    return output; // Return the output structure
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

	if (timer != NULL) {
		// read data in MPU6050 (library)
		MPU6050_Read_All(&hi2c1, &MPU6050);

		// store sync time in mpu6050_msg time stamp
		mpu6050_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000; // second unit
		mpu6050_msg.header.stamp.nanosec =  rmw_uros_epoch_nanos(); // nano second unit

		// store data MPU6050 in mpu6050_msg
		// 	unit m/s^2
		mpu6050_msg.linear_acceleration.x = G2M_S2 * MPU6050.Ax;
		mpu6050_msg.linear_acceleration.y = G2M_S2 * MPU6050.Ay;
		mpu6050_msg.linear_acceleration.z = G2M_S2 * MPU6050.Az;
		// unit rad/s
		mpu6050_msg.angular_velocity.x = DEG2RAD * MPU6050.Gx;
		mpu6050_msg.angular_velocity.y = DEG2RAD * MPU6050.Gy;
		mpu6050_msg.angular_velocity.z = DEG2RAD * MPU6050.Gz;
		// Test get Angle
		mpu6050_msg.orientation.x = MPU6050.KalmanAngleX;
		mpu6050_msg.orientation.y = MPU6050.KalmanAngleY;
		rcl_publish(&mpu6050_publisher, &mpu6050_msg, NULL);

		VelocityOutput velocities = convertRollPitchToVelocity(mpu6050_msg.orientation.x, mpu6050_msg.orientation.y);
		pub_msg.linear.x = velocities.linearVelocityX;
		pub_msg.angular.z = velocities.angularVelocityZ;
		rcl_publish(&cmd_vel_publisher, &pub_msg, NULL);

		// Auto Reconnect
		HAL_IWDG_Refresh(&hiwdg);
	}
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &hlpuart1,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app
  allocator = rcl_get_default_allocator();

  // Initialize and modify options (Set DOMAIN ID to 10)
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 23);

  //create init_options
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  rclc_node_init_default(&node, "cubemx_node", "", &support);

  // Sync time (for add in timestamp imu)
  rmw_uros_sync_session(1000); // ms unit

  // create timer
  rclc_timer_init_default(
		  &mpu6050_timer,
		  &support,
		  RCL_MS_TO_NS(10), // 1000 ms
		  timer_callback
		  );

  // create publisher
  rclc_publisher_init_best_effort(
		  &mpu6050_publisher,
		  &node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs ,msg ,Imu),
		  "mpu6050_publisher"
  );
  rclc_publisher_init_default(
		  &cmd_vel_publisher,
		  &node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs ,msg ,Twist),
		  "cmd_vel"
  );

  //create  message
  mpu6050_msg.header.frame_id = micro_ros_string_utilities_init("imu_frame");


  // create executor
  executor = rclc_executor_get_zero_initialized_executor(); // Get default config of executor
  unsigned int num_handles = 1; //num_handles คือ จำนวนของ callback ที่เรา handle ซึ่งตอนนี้มีคือ timer จึงเท่ากับ 1
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  // add timer (ถ้า add callback ใดก่อน callback นั้นจะเริ่มทำงานก่อน)
  rclc_executor_add_timer(&executor, &mpu6050_timer);
  // เรียกใช้ spin เพื่อให้ callback ต่างๆทำงาน
  rclc_executor_spin(&executor);

  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END 5 */
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // initialization of the GY-521 (MPU6050)
  while (MPU6050_Init(&hi2c1) == 1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
  if (htim->Instance == TIM1) {
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
