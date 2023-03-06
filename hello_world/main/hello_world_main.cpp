#include <Wire.h>

#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <nvs_flash.h>

#include "Adafruit_TCS34725.h"

#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
// #include <std_msgs/msg/u_int16.h>
#include <color_sensor_message/msg/color_sensor_message.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"

#include "app_log.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
color_sensor_message__msg__ColorSensorMessage msg;

/* initialization */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

extern "C" void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    color_sensor_message__msg__ColorSensorMessage__init(&msg);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (1) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    } else {
        msg.clear = 0;
        msg.red = 0;
        msg.green = 0;
        msg.blue = 0;
        msg.temperature = 0;
        printf("Failed to measure :(\n");
    }
  }
  // add delay to avoid overflow
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
  RCCHECK(rclc_node_init_default(&node, "sensors", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(color_sensor_message, msg, ColorSensorMessage),
    "tof_sensor_1"));

  // create timer;
  rcl_timer_t timer;
  const unsigned int timer_timeout = 1;

  /* initialization */

  // send data to callback function
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));


  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    // usleep(100);
  }

  // free resources
  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);

}


void publish_sensor_data(void *arg) {

  while (1)
  {
    color_sensor_message__msg__ColorSensorMessage__init(&msg);
    uint16_t clear, red, green, blue, colorTemp, lux;
    tcs.getRawData(&red, &green, &blue, &clear);
    colorTemp = tcs.calculateColorTemperature_dn40(red, green, blue, clear);
    lux = tcs.calculateLux(red, green, blue);

    // print everything in single line

    // printf("Color Temp: %d K - , Lux: %d - , R: %d , G: %d , B: %d , C: %d \n", colorTemp, lux, red, green, blue, clear);
    if (1) {
      msg.clear = clear;
      msg.red = red;
      msg.green = green;
      msg.blue = blue;
      msg.temperature = colorTemp;
    } else {
      printf("Failed to measure :(\n");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

extern "C" void app_main() {
  /* Boot Message */
  /* NVS flash initialization */
  nvs_flash_init();

  
#ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // UCLIENT_PROFILE_UDP

  logi;
  printf("Adafruit_TCS34725\n");

  /* NVS flash initialization */
  nvs_flash_init();

  
  if (!tcs.begin()) {
    loge;
    printf("Failed to initialize Adafruit_TCS34725\n");
    return;
  }

  xTaskCreate(
    publish_sensor_data,
    "sensor_publisher_task",
    16000,
    NULL,
    1,
    NULL
  );
  
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