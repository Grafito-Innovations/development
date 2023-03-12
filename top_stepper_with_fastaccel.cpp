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
#include <std_msgs/msg/u_int8.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"

#define DIR_PIN_GPIO_NUM    GPIO_NUM_18
#define ENABLE_PIN_GPIO_NUM GPIO_NUM_27
#define STEP_PIN_GPIO_NUM   GPIO_NUM_17

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

static const char* TAG  = "Stepper";

extern "C" void app_main() {
   gpio_pad_select_gpio(DIR_PIN_GPIO_NUM);
   gpio_pad_select_gpio(ENABLE_PIN_GPIO_NUM);
   gpio_pad_select_gpio(STEP_PIN_GPIO_NUM);

   gpio_set_direction(DIR_PIN_GPIO_NUM, GPIO_MODE_OUTPUT);
   gpio_set_direction(ENABLE_PIN_GPIO_NUM, GPIO_MODE_OUTPUT);
   gpio_set_direction(STEP_PIN_GPIO_NUM, GPIO_MODE_OUTPUT);

   engine.init();
   stepper = engine.stepperConnectToPin(STEP_PIN_GPIO_NUM);
   if (stepper) {
      stepper->setDirectionPin(DIR_PIN_GPIO_NUM);
      stepper->setEnablePin(ENABLE_PIN_GPIO_NUM);
      stepper->setAutoEnable(true);

      stepper->setSpeedInHz(500);       // 500 steps/s
      stepper->setAcceleration(100);    // 100 steps/s²
      // stepper->move(800); // move cannot be used to suddenly change the set point use moveTo instead
      // stepper->forceStopAndNewPosition(-800);
      // stepper->runForward();
      // stepper->runBackward();
      // stepper->moveTo(800);
      // stepper->moveTo(-800);
 
   while(1){
      stepper->moveTo(800);
      ESP_LOGI(TAG, "currentstep: %d", stepper->getCurrentPosition());
      vTaskDelay(1 / portTICK_PERIOD_MS);
   }
    
   }
}
