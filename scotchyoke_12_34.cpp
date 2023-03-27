#include <PID_v1.h>
#include "driver/ledc.h"
#include "esp32-hal-ledc.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"

#define LEDC_LOW_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_LOW_SPEED_TIMER LEDC_TIMER_0
#define LEDC_LOW_SPEED_FREQ_HZ 5000
#define LEDC_LOW_SPEED_RESOLUTION LEDC_TIMER_10_BIT

// SY-1/SY-3 put 300 to cut 0 to instial position
#define MOTOR_AOUT1 (gpio_num_t)4
#define MOTOR_AIN1 (gpio_num_t)13
#define ENCODER_A1 (gpio_num_t)32
#define ENCODER_B1 (gpio_num_t)23

// SY-2/SY-4
#define MOTOR_AOUT2 (gpio_num_t)19
#define MOTOR_AIN2 (gpio_num_t)18
#define ENCODER_A2 (gpio_num_t)34
#define ENCODER_B2 (gpio_num_t)35

// static const char* TAG  = "MOTOR";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher_sy;
std_msgs__msg__Int32MultiArray msg_sy;

int32_t data_sy[2] = {0, 0}; 

rcl_subscription_t subscriber_sy;
std_msgs__msg__Float32 recv_msg_sy;
double setpoint_sy = 0.0;


// Define PID constants
double Kp = 30;
double Ki = 500;
double Kd = 1.0;

// Define motor control variables
double motorInput1, motorOutput1, motorSetpoint_sy; 
double motorInput2, motorOutput2; 

// Define PID objects
PID motorPID1(&motorInput1, &motorOutput1, &motorSetpoint_sy, Kp, Ki, Kd, DIRECT);
PID motorPID2(&motorInput2, &motorOutput2, &motorSetpoint_sy, Kp, Ki, Kd, DIRECT);

typedef struct {
    int16_t pos;
    int8_t dir;
} Encoder;
Encoder encoder1 = {0, 1};
Encoder encoder2 = {0, 1};

int NewSetpoint_sy;
double MaxChange_sy = 5000;  // maximum change in setpoint_sy per second
double TimeInterval_sy = 5.0;  // time interval in milliseconds
double SetpointDelta_sy = MaxChange_sy * TimeInterval_sy / 5000.0;  // calculate the number of ticks to change the setpoint_sy in each time interval //edit 

void encoder_isr1(void *arg) {
    volatile int32_t a = gpio_get_level(ENCODER_A1);
    volatile int32_t b = gpio_get_level(ENCODER_B1);

    if (a == 1 && b == 0) {
        encoder1.pos += encoder1.dir;
    } else if (a == 1 && b == 1) {
        encoder1.pos -= encoder1.dir;
    } else if (a == 0 && b == 1) {
        encoder1.pos += encoder1.dir;
    } else if (a == 0 && b == 0) {
        encoder1.pos -= encoder1.dir;
    }}

void encoder_isr2(void *arg) {
    volatile int32_t a = gpio_get_level(ENCODER_A2);
    volatile int32_t b = gpio_get_level(ENCODER_B2);

    if (a == 1 && b == 0) {
        encoder2.pos += encoder2.dir;
    } else if (a == 1 && b == 1) {
        encoder2.pos -= encoder2.dir;
    } else if (a == 0 && b == 1) {
        encoder2.pos += encoder2.dir;
    } else if (a == 0 && b == 0) {
        encoder2.pos -= encoder2.dir;
    }}    

extern "C" void timer_callback_sy(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
       msg_sy.data.data[0]= encoder1.pos;
       msg_sy.data.data[1]= encoder2.pos;
		RCSOFTCHECK(rcl_publish(&publisher_sy, &msg_sy, NULL));
		
        // vTaskDelay(pdMS_TO_TICKS(1000));
	}}

extern "C" void subscription_callback_sy(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    NewSetpoint_sy = (int) msg->data;
    // ESP_LOGI(TAG, "Setpoint: %f", setpoint_sy);
}

// Define the task that updates the setpoint_sy
extern "C" void setpoint_task_sy(void *pvParameters)
{
    while (true)
    {

        // Calculate the number of time intervals required to reach the new setpoint_sy
        int NumIntervals_sy = abs(NewSetpoint_sy - setpoint_sy) / SetpointDelta_sy;

        // Gradually adjust the setpoint_sy in each time interval
        for (int i = 0; i < NumIntervals_sy; i++)
        {
            if (NewSetpoint_sy > setpoint_sy)
            {
                setpoint_sy += SetpointDelta_sy;
            }
            else if (NewSetpoint_sy < setpoint_sy)
            {
                setpoint_sy -= SetpointDelta_sy;
            }
            vTaskDelay(pdMS_TO_TICKS(TimeInterval_sy));
        }
    }
}

// Task to control motor using PID
extern "C" void motorTask1(void *pvParameters) {

  while (1) {
    
    // Set motor input to encoder1 value
    // ESP_LOGI(TAG, "encoderValue: %d", encoder1.pos);
    motorSetpoint_sy = setpoint_sy;
    motorInput1 = encoder1.pos;
    
    // Compute PID output
    motorPID1.Compute();
    // ESP_LOGI(TAG, "Motor output: %f", motorOutput1);
    // Set motor speed based on PID output
    if (motorOutput1 > 400) {
      gpio_set_level(MOTOR_AOUT1, 1);
      gpio_set_level(MOTOR_AIN1, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (int)motorOutput1);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
    else if (motorOutput1 < -400) {
      gpio_set_level(MOTOR_AOUT1, 0);
      gpio_set_level(MOTOR_AIN1, 1);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (int)-motorOutput1);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    else {
      gpio_set_level(MOTOR_AOUT1, 0);
      gpio_set_level(MOTOR_AIN1, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// Task to control motor using PID
extern "C" void motorTask2(void *pvParameters) {

  while (1) {
    
    // Set motor input to encoder1 value
    // ESP_LOGI(TAG, "encoderValue: %d", encoder1.pos);
    motorSetpoint_sy = setpoint_sy;
    motorInput2 = encoder2.pos;
    
    // Compute PID output
    motorPID2.Compute();
    // ESP_LOGI(TAG, "Motor output: %f", motorOutput1);
    // Set motor speed based on PID output
    if (motorOutput2 > 400) {
      gpio_set_level(MOTOR_AOUT2, 1);
      gpio_set_level(MOTOR_AIN2, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, (int)motorOutput2);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    }
    else if (motorOutput2 < -400) {
      gpio_set_level(MOTOR_AOUT2, 0);
      gpio_set_level(MOTOR_AIN2, 1);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (int)-motorOutput2);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }
    else {
      gpio_set_level(MOTOR_AOUT2, 0);
      gpio_set_level(MOTOR_AIN2, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

extern "C" void wifiTask(void *pvParameters) {

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node ;
	RCCHECK(rclc_node_init_default(&node, "motor_info", "", &support));

	// create publisher_sy
	RCCHECK(rclc_publisher_init_default(
		&publisher_sy,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
		"sy_encoder"));

    // Create subscriber_sy.
	RCCHECK(rclc_subscription_init_default(
		&subscriber_sy,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"sy_setpoint")); 

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback_sy));


	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    // unsigned int rcl_wait_timeout = 1000;   // in ms
	// RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_sy, &recv_msg_sy, &subscription_callback_sy, ON_NEW_DATA));

	msg_sy.data.size = 2;
  msg_sy.data.data = data_sy;	

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(1000);
	}

	// free resources
  RCCHECK(rcl_subscription_fini(&subscriber_sy, &node));
	RCCHECK(rcl_publisher_fini(&publisher_sy, &node));
	RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);
}

extern "C" void app_main() {

  #ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
 #endif  // UCLIENT_PROFILE_UDP
  // Initialize motor pins
  gpio_pad_select_gpio(MOTOR_AOUT1);
  gpio_pad_select_gpio(MOTOR_AIN1);
  gpio_pad_select_gpio(MOTOR_AOUT2);
  gpio_pad_select_gpio(MOTOR_AIN2);
  gpio_set_direction(MOTOR_AOUT1, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTOR_AIN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTOR_AOUT2, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTOR_AIN2, GPIO_MODE_OUTPUT);

  // Configure low-speed LEDC channel
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_LOW_SPEED_RESOLUTION,
        .timer_num = LEDC_LOW_SPEED_TIMER,
        .freq_hz = LEDC_LOW_SPEED_FREQ_HZ,
    };
    ledc_timer_config(&ledc_timer);

    // Set duty cycle for low-speed LEDC channel
    ledc_channel_config_t ledc_channel0 = {
        .gpio_num = MOTOR_AIN1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_LOW_SPEED_TIMER,
        .duty = 0,
    };
    ledc_channel_config(&ledc_channel0);

    ledc_channel_config_t ledc_channel1 = {
        .gpio_num = MOTOR_AOUT1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_LOW_SPEED_TIMER,
        .duty = 0,
        
    };
    ledc_channel_config(&ledc_channel1);

    ledc_channel_config_t ledc_channel2 = {
        .gpio_num = MOTOR_AIN2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_LOW_SPEED_TIMER,
        .duty = 0,
    };
    ledc_channel_config(&ledc_channel2);

    ledc_channel_config_t ledc_channel3 = {
        .gpio_num = MOTOR_AOUT2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .timer_sel = LEDC_LOW_SPEED_TIMER,
        .duty = 0,
    };
    ledc_channel_config(&ledc_channel3);
  
  // Initialize PID objects
  motorPID1.SetMode(AUTOMATIC);
  motorPID1.SetSampleTime(2);
  motorPID1.SetOutputLimits(-1023, 1023);

  motorPID2.SetMode(AUTOMATIC);
  motorPID2.SetSampleTime(2);
  motorPID2.SetOutputLimits(-1023, 1023);

    gpio_pad_select_gpio(ENCODER_A1);
    gpio_pad_select_gpio(ENCODER_B1); 
    gpio_pad_select_gpio(ENCODER_A2);
    gpio_pad_select_gpio(ENCODER_B2);
  
    gpio_set_direction(ENCODER_B1, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_A1, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_B2, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_A2, GPIO_MODE_INPUT);

    gpio_set_intr_type(ENCODER_A1, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER_A2, GPIO_INTR_ANYEDGE);

    gpio_set_pull_mode(ENCODER_A1, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B1, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_A2, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B2, GPIO_PULLUP_ONLY);
   
    gpio_install_isr_service(0); 

    gpio_isr_handler_add(ENCODER_A1, encoder_isr1, (void*) ENCODER_A1);
    gpio_isr_handler_add(ENCODER_A2, encoder_isr2, (void*) ENCODER_A2);
    
    
  xTaskCreate(motorTask1, "motorTask1", 10000, NULL, 1, NULL);
  xTaskCreate(motorTask2, "motorTask2", 10000, NULL, 1, NULL);  
  xTaskCreate(setpoint_task_sy, "setpoint_task_sy", 5000, NULL, 1, NULL);
  xTaskCreate(wifiTask, "wifiTask", 5000, NULL, 1, NULL);
}