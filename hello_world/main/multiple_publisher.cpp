#define DUMP_REGS
 
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <nvs_flash.h>


#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t first_publisher;
rcl_publisher_t second_publisher;

std_msgs__msg__String message_one;
std_msgs__msg__String message_two;

extern "C" void timer_callback_one(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    std_msgs__msg__String__init(&message_one);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    message_one.data = "Hello from first publisher";
    RCSOFTCHECK(rcl_publish(&first_publisher, &message_one, NULL));
  }
}

extern "C" void timer_callback_two(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    std_msgs__msg__String__init(&message_two);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    message_two.data = "Hi from second publisher";
    RCSOFTCHECK(rcl_publish(&second_publisher, &message_two, NULL));
  }
}

extern "C" void micro_ros_task(void *arg)
{

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  rcl_node_t node;
  // create node "sensors" with a topic for "tof_sensor_1"
  RCCHECK(rclc_node_init_default(&node, "publisher", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &first_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "topic_one"));

  RCCHECK(rclc_publisher_init_default(
    &second_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "topic_two"));

  // create timer;
  rcl_timer_t timer_one;
  const unsigned int timer_timeout_one = 1;

  /* initialization */

  // send data to callback function
  RCCHECK(rclc_timer_init_default(
    &timer_one,
    &support,
    RCL_MS_TO_NS(timer_timeout_one),
    timer_callback));

  rcl_timer_t timer_two;
  const unsigned int timer_timeout_two = 1;

  RCCHECK(rclc_timer_init_default(
    &timer_two,
    &support,
    RCL_MS_TO_NS(timer_timeout_two),
    timer_callback_two));

  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // add timer to executor  
  RCCHECK(rclc_executor_add_timer(&executor, &timer_one));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_two));

  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    usleep(100);
  }

  // free resources
  RCCHECK(rcl_publisher_fini(&first_publisher, &node));
  RCCHECK(rcl_publisher_fini(&second_publisher, &node));
  RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);

}

extern "C" void app_main(){
      /* Boot Message */
  /* NVS flash initialization */
  nvs_flash_init();

  
#ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // UCLIENT_PROFILE_UDP

  xTaskCreate(
    micro_ros_task, 
    "micro_ros_task", 
    16000,
    NULL,
    1,
    NULL
    );
 
  vTaskDelay(portMAX_DELAY);
}