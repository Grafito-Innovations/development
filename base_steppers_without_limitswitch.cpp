#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FastAccelStepper.h"
#include "driver/gpio.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_system.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define STEP_PIN_GPIO_NUM   GPIO_NUM_21
#define DIR_PIN_GPIO_NUM    GPIO_NUM_17
#define ENABLE_PIN_GPIO_NUM GPIO_NUM_18

#define STEP_PIN_GPIO_NUM2  GPIO_NUM_25
#define DIR_PIN_GPIO_NUM2   GPIO_NUM_26
#define ENABLE_PIN_GPIO_NUM2 GPIO_NUM_27

#define STEP_PIN_GPIO_NUM3  GPIO_NUM_32
#define DIR_PIN_GPIO_NUM3   GPIO_NUM_33
#define ENABLE_PIN_GPIO_NUM3 GPIO_NUM_16

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;

static const char* TAG  = "Stepper";

rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray msg;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 recv_msg;
rcl_subscription_t subscriber3;
std_msgs__msg__Int32 recv_msg3;
rcl_subscription_t subscriber_set_speed;
std_msgs__msg__Int32 speed_msg;
rcl_subscription_t subscriber_set_accel;
std_msgs__msg__Int32 accel_msg;

int32_t data[3] = {0, 0, 0}; 

int current_step_position;
int current_step_position2;
int current_step_position3;
int NewSetpoint = 0; // intial pos to 0 (make it near to the bottom), at far end -835000 for base not for top stepper
int newSpeedInHz = 2000;
int newAcceleration = 1000;

int NewSetpoint3 = 0; // intial pos to 0 (make it near to the bottom), at far end -845000 for base not for top stepper

extern "C" void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
       msg.data.data[0]= current_step_position;
       msg.data.data[1]= current_step_position2;
       msg.data.data[2]= current_step_position3;
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		
        // vTaskDelay(pdMS_TO_TICKS(1000));
	}}
    
extern "C" void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    NewSetpoint = msg->data;
    // ESP_LOGI(TAG, "Setpoint: %f", setpoint);
}

extern "C" void subscription_callback3(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    NewSetpoint3 = msg->data;
    // ESP_LOGI(TAG, "Setpoint: %f", setpoint);
}

extern "C" void subscription_callback_setSpeed(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    newSpeedInHz = msg->data;
    // ESP_LOGI(TAG, "Setpoint: %f", setpoint);
}

extern "C" void subscription_callback_setAccel(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    newAcceleration = msg->data;
    // ESP_LOGI(TAG, "Setpoint: %f", setpoint);
}

extern "C" void topstepperTask(void *pvParameters) {
   gpio_pad_select_gpio(DIR_PIN_GPIO_NUM);
   gpio_pad_select_gpio(ENABLE_PIN_GPIO_NUM);
   gpio_pad_select_gpio(STEP_PIN_GPIO_NUM);

   gpio_set_direction(DIR_PIN_GPIO_NUM, GPIO_MODE_OUTPUT);
   gpio_set_direction(ENABLE_PIN_GPIO_NUM, GPIO_MODE_OUTPUT);
   gpio_set_direction(STEP_PIN_GPIO_NUM, GPIO_MODE_OUTPUT);

   gpio_pad_select_gpio(DIR_PIN_GPIO_NUM2);
   gpio_pad_select_gpio(ENABLE_PIN_GPIO_NUM2);
   gpio_pad_select_gpio(STEP_PIN_GPIO_NUM2);

   gpio_set_direction(DIR_PIN_GPIO_NUM2, GPIO_MODE_OUTPUT);
   gpio_set_direction(ENABLE_PIN_GPIO_NUM2, GPIO_MODE_OUTPUT);
   gpio_set_direction(STEP_PIN_GPIO_NUM2, GPIO_MODE_OUTPUT);

   gpio_pad_select_gpio(DIR_PIN_GPIO_NUM3);
   gpio_pad_select_gpio(ENABLE_PIN_GPIO_NUM3);
   gpio_pad_select_gpio(STEP_PIN_GPIO_NUM3);

   gpio_set_direction(DIR_PIN_GPIO_NUM3, GPIO_MODE_OUTPUT);
   gpio_set_direction(ENABLE_PIN_GPIO_NUM3, GPIO_MODE_OUTPUT);
   gpio_set_direction(STEP_PIN_GPIO_NUM3, GPIO_MODE_OUTPUT);

   stepper = engine.stepperConnectToPin(STEP_PIN_GPIO_NUM);
   stepper2 = engine.stepperConnectToPin(STEP_PIN_GPIO_NUM2);
   stepper3 = engine.stepperConnectToPin(STEP_PIN_GPIO_NUM3);
   if (stepper and stepper2 and stepper3) {
      stepper->setDirectionPin(DIR_PIN_GPIO_NUM);
      stepper->setEnablePin(ENABLE_PIN_GPIO_NUM);
      stepper->setAutoEnable(true);

      stepper2->setDirectionPin(DIR_PIN_GPIO_NUM2);
      stepper2->setEnablePin(ENABLE_PIN_GPIO_NUM2);
      stepper2->setAutoEnable(true); 

      stepper3->setDirectionPin(DIR_PIN_GPIO_NUM3);
      stepper3->setEnablePin(ENABLE_PIN_GPIO_NUM3);
      stepper3->setAutoEnable(true); 

    while(1){

      stepper->setSpeedInHz(newSpeedInHz);       // 500 steps/s
      stepper->setAcceleration(newAcceleration);    // 100 steps/s??  
      stepper->moveTo(NewSetpoint);
      current_step_position = stepper->getCurrentPosition();
    //   ESP_LOGI(TAG, "currentstep: %d", current_step_position);

      stepper2->setSpeedInHz(newSpeedInHz);       // 500 steps/s
      stepper2->setAcceleration(newAcceleration);    // 100 steps/s??  
      stepper2->moveTo(NewSetpoint);
      current_step_position2 = stepper2->getCurrentPosition();
    //   ESP_LOGI(TAG, "currentstep: %d", current_step_position);

      stepper3->setSpeedInHz(newSpeedInHz);       // 500 steps/s
      stepper3->setAcceleration(newAcceleration);    // 100 steps/s??  
      stepper3->moveTo(NewSetpoint3);
      current_step_position3 = stepper3->getCurrentPosition();
    //   ESP_LOGI(TAG, "currentstep: %d", current_step_position);

      vTaskDelay(1 / portTICK_PERIOD_MS); 
   }
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
	RCCHECK(rclc_node_init_default(&node, "top_stepper_info", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
		"base_stepper_currentpos_insteps"));

    // Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"base_xstepper_new_setpoint_insteps")); 

    // Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber3,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"base_ystepper_new_setpoint_insteps")); 

        // Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber_set_speed,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"base_stepper_set_speed_in_hz")); 

        // Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber_set_accel,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"base_stepper_set_acceleration"));         

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));


	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    // unsigned int rcl_wait_timeout = 1000;   // in ms
	// RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber3, &recv_msg3, &subscription_callback3, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_set_speed, &speed_msg, &subscription_callback_setSpeed, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_set_accel, &accel_msg, &subscription_callback_setAccel, ON_NEW_DATA));

	msg.data.size = 3;
    msg.data.data = data;	

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(1000);
	}

	// free resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_subscription_fini(&subscriber3, &node));    
    RCCHECK(rcl_subscription_fini(&subscriber_set_speed, &node));
    RCCHECK(rcl_subscription_fini(&subscriber_set_accel, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

extern "C" void app_main() {

  #ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
 #endif  // UCLIENT_PROFILE_UDP

  xTaskCreate(topstepperTask, "topstepperTask", 20000, NULL, 1, NULL); 
  xTaskCreate(wifiTask, "wifiTask", 10000, NULL, 1, NULL);
  engine.init();
}
