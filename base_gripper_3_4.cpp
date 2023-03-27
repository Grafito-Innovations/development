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

// BG-3 
#define MOTOR_AOUT3 (gpio_num_t)21
#define MOTOR_AIN3 (gpio_num_t)22
#define ENCODER_A3 (gpio_num_t)39
#define ENCODER_B3 (gpio_num_t)36

// BG-4
#define MOTOR_AOUT4 (gpio_num_t)25
#define MOTOR_AIN4 (gpio_num_t)33
#define ENCODER_A4 (gpio_num_t)26
#define ENCODER_B4 (gpio_num_t)27

// static const char* TAG  = "MOTOR";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher_bg;
std_msgs__msg__Int32MultiArray msg_bg;

int32_t data_bg[2] = {0, 0}; 

rcl_subscription_t subscriber_bg;
std_msgs__msg__Float32 recv_msg_bg;
double setpoint_bg = 0.0;


// Define PID constants
double Kp = 30;
double Ki = 500;
double Kd = 1.0;

// Define motor control variables
double motorInput3, motorOutput3, motorSetpoint_bg; 
double motorInput4, motorOutput4; 

// Define PID objects
PID motorPID3(&motorInput3, &motorOutput3, &motorSetpoint_bg, Kp, Ki, Kd, DIRECT);
PID motorPID4(&motorInput4, &motorOutput4, &motorSetpoint_bg, Kp, Ki, Kd, DIRECT);

typedef struct {
    int16_t pos;
    int8_t dir;
} Encoder;
Encoder encoder3 = {0, 1};
Encoder encoder4 = {0, 1};

int NewSetpoint_bg;
double MaxChange_bg = 5000;  // maximum change in setpoint_bg per second
double TimeInterval_bg = 5.0;  // time interval in milliseconds
double SetpointDelta_bg = MaxChange_bg * TimeInterval_bg / 5000.0;  // calculate the number of ticks to change the setpoint_bg in each time interval //edit 

void encoder_isr3(void *arg) {
    volatile int32_t a = gpio_get_level(ENCODER_A3);
    volatile int32_t b = gpio_get_level(ENCODER_B3);

    if (a == 1 && b == 0) {
        encoder3.pos += encoder3.dir;
    } else if (a == 1 && b == 1) {
        encoder3.pos -= encoder3.dir;
    } else if (a == 0 && b == 1) {
        encoder3.pos += encoder3.dir;
    } else if (a == 0 && b == 0) {
        encoder3.pos -= encoder3.dir;
    }}

void encoder_isr4(void *arg) {
    volatile int32_t a = gpio_get_level(ENCODER_A4);
    volatile int32_t b = gpio_get_level(ENCODER_B4);

    if (a == 1 && b == 0) {
        encoder4.pos += encoder4.dir;
    } else if (a == 1 && b == 1) {
        encoder4.pos -= encoder4.dir;
    } else if (a == 0 && b == 1) {
        encoder4.pos += encoder4.dir;
    } else if (a == 0 && b == 0) {
        encoder4.pos -= encoder4.dir;
    }}    

extern "C" void timer_callback_bg(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
       msg_bg.data.data[0]= encoder3.pos;
       msg_bg.data.data[1]= encoder4.pos;
		RCSOFTCHECK(rcl_publish(&publisher_bg, &msg_bg, NULL));
		
        // vTaskDelay(pdMS_TO_TICKS(1000));
	}}

extern "C" void subscription_callback_bg(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    NewSetpoint_bg = (int) msg->data;
    // ESP_LOGI(TAG, "Setpoint: %f", setpoint_bg);
}

// Define the task that updates the setpoint_bg
extern "C" void setpoint_task_bg(void *pvParameters)
{
    while (true)
    {

        // Calculate the number of time intervals required to reach the new setpoint_bg
        int NumIntervals_bg = abs(NewSetpoint_bg - setpoint_bg) / SetpointDelta_bg;

        // Gradually adjust the setpoint_bg in each time interval
        for (int i = 0; i < NumIntervals_bg; i++)
        {
            if (NewSetpoint_bg > setpoint_bg)
            {
                setpoint_bg += SetpointDelta_bg;
            }
            else if (NewSetpoint_bg < setpoint_bg)
            {
                setpoint_bg -= SetpointDelta_bg;
            }
            vTaskDelay(pdMS_TO_TICKS(TimeInterval_bg));
        }
    }
}

// Task to control motor using PID
extern "C" void motorTask3(void *pvParameters) {

  while (1) {
    
    // Set motor input to encoder3 value
    // ESP_LOGI(TAG, "encoderValue: %d", encoder3.pos);
    motorSetpoint_bg = setpoint_bg;
    motorInput3 = encoder3.pos;
    
    // Compute PID output
    motorPID3.Compute();
    // ESP_LOGI(TAG, "Motor output: %f", motorOutput3);
    // Set motor speed based on PID output
    if (motorOutput3 > 400) {
      gpio_set_level(MOTOR_AOUT3, 1);
      gpio_set_level(MOTOR_AIN3, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, (int)motorOutput3);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
    }
    else if (motorOutput3 < -400) {
      gpio_set_level(MOTOR_AOUT3, 0);
      gpio_set_level(MOTOR_AIN3, 1);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, (int)-motorOutput3);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
    }
    else {
      gpio_set_level(MOTOR_AOUT3, 0);
      gpio_set_level(MOTOR_AIN3, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 0);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// Task to control motor using PID
extern "C" void motorTask4(void *pvParameters) {

  while (1) {
    
    // Set motor input to encoder3 value
    // ESP_LOGI(TAG, "encoderValue: %d", encoder3.pos);
    motorSetpoint_bg = setpoint_bg;
    motorInput4 = encoder4.pos;
    
    // Compute PID output
    motorPID4.Compute();
    // ESP_LOGI(TAG, "Motor output: %f", motorOutput3);
    // Set motor speed based on PID output
    if (motorOutput4 > 400) {
      gpio_set_level(MOTOR_AOUT4, 1);
      gpio_set_level(MOTOR_AIN4, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, (int)motorOutput4);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7);
    }
    else if (motorOutput4 < -400) {
      gpio_set_level(MOTOR_AOUT4, 0);
      gpio_set_level(MOTOR_AIN4, 1);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, (int)-motorOutput4);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);
    }
    else {
      gpio_set_level(MOTOR_AOUT4, 0);
      gpio_set_level(MOTOR_AIN4, 0);
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, 0);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);
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

	// create publisher_bg
	RCCHECK(rclc_publisher_init_default(
		&publisher_bg,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
		"bg_encoder"));

    // Create subscriber_bg.
	RCCHECK(rclc_subscription_init_default(
		&subscriber_bg,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"bg_setpoint")); 

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback_bg));


	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    // unsigned int rcl_wait_timeout = 1000;   // in ms
	// RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_bg, &recv_msg_bg, &subscription_callback_bg, ON_NEW_DATA));

	msg_bg.data.size = 2;
  msg_bg.data.data = data_bg;	

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(1000);
	}

	// free resources
  RCCHECK(rcl_subscription_fini(&subscriber_bg, &node));
	RCCHECK(rcl_publisher_fini(&publisher_bg, &node));
	RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);
}

extern "C" void app_main() {

  #ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
 #endif  // UCLIENT_PROFILE_UDP
  // Initialize motor pins
  gpio_pad_select_gpio(MOTOR_AOUT3);
  gpio_pad_select_gpio(MOTOR_AIN3);
  gpio_pad_select_gpio(MOTOR_AOUT4);
  gpio_pad_select_gpio(MOTOR_AIN4);
  gpio_set_direction(MOTOR_AOUT3, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTOR_AIN3, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTOR_AOUT4, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTOR_AIN4, GPIO_MODE_OUTPUT);

  // Configure low-speed LEDC channel
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_LOW_SPEED_RESOLUTION,
        .timer_num = LEDC_LOW_SPEED_TIMER,
        .freq_hz = LEDC_LOW_SPEED_FREQ_HZ,
    };
    ledc_timer_config(&ledc_timer);

    // Set duty cycle for low-speed LEDC channel
    ledc_channel_config_t ledc_channel4 = {
        .gpio_num = MOTOR_AIN3,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_LOW_SPEED_TIMER,
        .duty = 0,
    };
    ledc_channel_config(&ledc_channel4);

    ledc_channel_config_t ledc_channel5 = {
        .gpio_num = MOTOR_AOUT3,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_5,
        .timer_sel = LEDC_LOW_SPEED_TIMER,
        .duty = 0,
        
    };
    ledc_channel_config(&ledc_channel5);

    ledc_channel_config_t ledc_channel6 = {
        .gpio_num = MOTOR_AIN4,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_6,
        .timer_sel = LEDC_LOW_SPEED_TIMER,
        .duty = 0,
    };
    ledc_channel_config(&ledc_channel6);

    ledc_channel_config_t ledc_channel7 = {
        .gpio_num = MOTOR_AOUT4,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_7,
        .timer_sel = LEDC_LOW_SPEED_TIMER,
        .duty = 0,
    };
    ledc_channel_config(&ledc_channel7);
  
  // Initialize PID objects
  motorPID3.SetMode(AUTOMATIC);
  motorPID3.SetSampleTime(2);
  motorPID3.SetOutputLimits(-1023, 1023);

  motorPID4.SetMode(AUTOMATIC);
  motorPID4.SetSampleTime(2);
  motorPID4.SetOutputLimits(-1023, 1023);

    gpio_pad_select_gpio(ENCODER_A3);
    gpio_pad_select_gpio(ENCODER_B3); 
    gpio_pad_select_gpio(ENCODER_A4);
    gpio_pad_select_gpio(ENCODER_B4);
  
    gpio_set_direction(ENCODER_B3, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_A3, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_B4, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_A4, GPIO_MODE_INPUT);

    gpio_set_intr_type(ENCODER_A3, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER_A4, GPIO_INTR_ANYEDGE);

    gpio_set_pull_mode(ENCODER_A3, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B3, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_A4, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B4, GPIO_PULLUP_ONLY);
   
    gpio_install_isr_service(0); 

    gpio_isr_handler_add(ENCODER_A3, encoder_isr3, (void*) ENCODER_A3);
    gpio_isr_handler_add(ENCODER_A4, encoder_isr4, (void*) ENCODER_A4);
    
    
  xTaskCreate(motorTask3, "motorTask3", 10000, NULL, 1, NULL);
  xTaskCreate(motorTask4, "motorTask4", 10000, NULL, 1, NULL);  
  xTaskCreate(setpoint_task_bg, "setpoint_task_bg", 5000, NULL, 1, NULL);
  xTaskCreate(wifiTask, "wifiTask", 5000, NULL, 1, NULL);
}