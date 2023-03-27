#include <PID_v1.h>
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
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

rcl_publisher_t publisher_sy;
std_msgs__msg__Int32MultiArray msg_sy;

int32_t data_sy[2] = {0, 0}; 

rcl_subscription_t subscriber_sy;
std_msgs__msg__Float32 recv_msg_sy;
double setpoint_sy = 0.0;

rcl_publisher_t publisher_bg;
std_msgs__msg__Int32MultiArray msg_bg;

int32_t data_bg[2] = {0, 0}; 

rcl_subscription_t subscriber_bg;
std_msgs__msg__Float32 recv_msg_bg;
double setpoint_bg = 0.0;

// Define PID constants
double Kp_sy = 30;
double Ki_sy = 400;
double Kd_sy = 1.0;

double Kp_bg = 30;
double Ki_bg = 500;
double Kd_bg = 1.0;

// Define motor control variables
double motorInput1, motorOutput1, motorSetpoint_sy; 
double motorInput2, motorOutput2; 
double motorInput3, motorOutput3, motorSetpoint_bg; 
double motorInput4, motorOutput4; 

// Define PID objects
PID motorPID1(&motorInput1, &motorOutput1, &motorSetpoint_sy, Kp_sy, Ki_sy, Kd_sy, DIRECT);
PID motorPID2(&motorInput2, &motorOutput2, &motorSetpoint_sy, Kp_sy, Ki_sy, Kd_sy, DIRECT);
PID motorPID3(&motorInput3, &motorOutput3, &motorSetpoint_bg, Kp_bg, Ki_bg, Kd_bg, DIRECT);
PID motorPID4(&motorInput4, &motorOutput4, &motorSetpoint_bg, Kp_bg, Ki_bg, Kd_bg, DIRECT);

typedef struct {
    int16_t pos;
    int8_t dir;
} Encoder;
Encoder encoder1 = {0, 1};
Encoder encoder2 = {0, 1};
Encoder encoder3 = {0, 1};
Encoder encoder4 = {0, 1};

int NewSetpoint_sy;
double MaxChange_sy = 10000;  // maximum change in setpoint_sy per second
double TimeInterval_sy = 1.0;  // time interval in milliseconds
double SetpointDelta_sy = MaxChange_sy * TimeInterval_sy / 5000.0;  // calculate the number of ticks to change the setpoint_sy in each time interval //edit 

int NewSetpoint_bg;
double MaxChange_bg = 10000;  // maximum change in setpoint_bg per second
double TimeInterval_bg = 1.0;  // time interval in milliseconds
double SetpointDelta_bg = MaxChange_bg * TimeInterval_bg / 5000.0;  // calculate the number of ticks to change the setpoint_bg in each time interval //edit 

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


extern "C" void timer_callback_sy(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
       msg_sy.data.data[0]= encoder1.pos;
       msg_sy.data.data[1]= encoder2.pos;
		RCSOFTCHECK(rcl_publish(&publisher_sy, &msg_sy, NULL));
		
        // vTaskDelay(pdMS_TO_TICKS(1000));
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

extern "C" void subscription_callback_sy(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    NewSetpoint_sy = (int) msg->data;
    // ESP_LOGI(TAG, "Setpoint: %f", setpoint_sy);
}

extern "C" void subscription_callback_bg(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    NewSetpoint_bg = (int) msg->data;
    // ESP_LOGI(TAG, "Setpoint: %f", setpoint_bg);
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
        vTaskDelay(pdMS_TO_TICKS(1));
    }
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
        vTaskDelay(pdMS_TO_TICKS(1));
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
    if (motorOutput1 > 300) {
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (int)motorOutput1);
    }
    else if (motorOutput1 < -300) {
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (int)-motorOutput1);
    }
    else {
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
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
    if (motorOutput2 > 300) {
      mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, (int)motorOutput2);
    }
    else if (motorOutput2 < -300) {
      mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, (int)-motorOutput2);
    }
    else {
      mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
      mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
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

    vTaskDelay(3 / portTICK_PERIOD_MS);
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

    vTaskDelay(3 / portTICK_PERIOD_MS);
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
	RCCHECK(rclc_node_init_default(&node, "scotchyoke_basegripper", "", &support));

	// create publisher_sy
	RCCHECK(rclc_publisher_init_default(
		&publisher_sy,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
		"sy_encoder"));

    // create publisher_bg
	RCCHECK(rclc_publisher_init_default(
		&publisher_bg,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
		"bg_encoder"));

    // Create subscriber_sy.
	RCCHECK(rclc_subscription_init_default(
		&subscriber_sy,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"sy_setpoint")); 

    // Create subscriber_bg.
	RCCHECK(rclc_subscription_init_default(
		&subscriber_bg,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"bg_setpoint"));     

	// create timer,
	rcl_timer_t timer_sy;
	const unsigned int timer_timeout_sy = 10;
	RCCHECK(rclc_timer_init_default(
		&timer_sy,
		&support,
		RCL_MS_TO_NS(timer_timeout_sy),
		timer_callback_sy));

	rcl_timer_t timer_bg;
	const unsigned int timer_timeout_bg = 10;
  RCCHECK(rclc_timer_init_default(
      &timer_bg,
      &support,
      RCL_MS_TO_NS(timer_timeout_bg),
      timer_callback_bg));


	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    // unsigned int rcl_wait_timeout = 1000;   // in ms
	// RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_timer(&executor, &timer_sy));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_bg));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_sy, &recv_msg_sy, &subscription_callback_sy, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_bg, &recv_msg_bg, &subscription_callback_bg, ON_NEW_DATA));    

	msg_sy.data.size = 2;
  msg_sy.data.data = data_sy;
	msg_bg.data.size = 2;
  msg_bg.data.data = data_bg;		

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(1000);
	}

	// free resources
  RCCHECK(rcl_subscription_fini(&subscriber_sy, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_bg, &node));
	RCCHECK(rcl_publisher_fini(&publisher_sy, &node));
	RCCHECK(rcl_publisher_fini(&publisher_bg, &node));    
	RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);
}

extern "C" void app_main() {

  #ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
 #endif  // UCLIENT_PROFILE_UDP

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


// Initialize the MCPWM units and timers for both motors
mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_AOUT1);
mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_AIN1);
mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, MOTOR_AOUT2);
mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, MOTOR_AIN2);

// Configure the MCPWM timers for both motors
mcpwm_config_t pwm_config;
pwm_config.frequency = 10000; // Frequency in Hz
pwm_config.cmpr_a = 50.0; // Initial duty cycle for PWM0A
pwm_config.cmpr_b = 50.0; // Initial duty cycle for PWM0B
pwm_config.counter_mode = MCPWM_UP_COUNTER;
pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

  // Initialize PID objects
  motorPID1.SetMode(AUTOMATIC);
  motorPID1.SetSampleTime(2);
  motorPID1.SetOutputLimits(-1023, 1023);

  motorPID2.SetMode(AUTOMATIC);
  motorPID2.SetSampleTime(2);
  motorPID2.SetOutputLimits(-1023, 1023);

  motorPID3.SetMode(AUTOMATIC);
  motorPID3.SetSampleTime(2);
  motorPID3.SetOutputLimits(-1023, 1023);

  motorPID4.SetMode(AUTOMATIC);
  motorPID4.SetSampleTime(2);
  motorPID4.SetOutputLimits(-1023, 1023);

    gpio_pad_select_gpio(ENCODER_A1);
    gpio_pad_select_gpio(ENCODER_B1); 
    gpio_pad_select_gpio(ENCODER_A2);
    gpio_pad_select_gpio(ENCODER_B2);
    gpio_pad_select_gpio(ENCODER_A3);
    gpio_pad_select_gpio(ENCODER_B3); 
    gpio_pad_select_gpio(ENCODER_A4);
    gpio_pad_select_gpio(ENCODER_B4);    
 
    gpio_set_direction(ENCODER_B1, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_A1, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_B2, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_A2, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_B3, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_A3, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_B4, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_A4, GPIO_MODE_INPUT);    

    gpio_set_intr_type(ENCODER_A1, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER_A2, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER_A3, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER_A4, GPIO_INTR_ANYEDGE);    

    gpio_set_pull_mode(ENCODER_A1, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B1, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_A2, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B2, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_A3, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B3, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_A4, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B4, GPIO_PULLUP_ONLY);    
   
    gpio_install_isr_service(0); 

    gpio_isr_handler_add(ENCODER_A1, encoder_isr1, (void*) ENCODER_A1);
    gpio_isr_handler_add(ENCODER_A2, encoder_isr2, (void*) ENCODER_A2);
    gpio_isr_handler_add(ENCODER_A3, encoder_isr3, (void*) ENCODER_A3);
    gpio_isr_handler_add(ENCODER_A4, encoder_isr4, (void*) ENCODER_A4);    
    
    
  xTaskCreate(motorTask1, "motorTask1", 7000, NULL, 1, NULL);
  xTaskCreate(motorTask2, "motorTask2", 7000, NULL, 1, NULL);  
  xTaskCreate(motorTask3, "motorTask3", 7000, NULL, 2, NULL);
  xTaskCreate(motorTask4, "motorTask4", 7000, NULL, 2, NULL);  
  xTaskCreate(setpoint_task_sy, "setpoint_task_sy", 7000, NULL, 0, NULL);
  xTaskCreate(setpoint_task_bg, "setpoint_task_bg", 7000, NULL, 0, NULL); 
  xTaskCreate(wifiTask, "wifiTask", 10000, NULL, 0, NULL);
}