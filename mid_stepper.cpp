#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "AccelStepper.h"

#define motorPin1  21
#define motorPin2  22
#define motorPin3  23
#define motorPin4  19

int a = 1;

AccelStepper stepper(AccelStepper::HALF4WIRE, motorPin1, motorPin3, motorPin2, motorPin4);
// rotate it forward by giving negative value (clockwise)

extern "C" void topstepperTask(void *pvParameters) 
{
// printf("Stepper motor control using ESP32\n");

    // Set up stepper motor
    stepper.setMaxSpeed(2500.0);
    stepper.setAcceleration(105000.0);
    stepper.setSpeed(2500);

    while (1) {
   
        printf("cur pos %ld \n",stepper.currentPosition());   

        if (stepper.currentPosition()>= 5000){
            a=2;
        }

        if (a==2){
            stepper.moveTo(0);
            stepper.run();
        }
        else {
            stepper.moveTo(10048);
            stepper.run();
   
        }

      vTaskDelay(1 / portTICK_PERIOD_MS);

    //     stepper.move(-10048);
    //     while (stepper.distanceToGo() != 0) {
    //         stepper.run();
    //         vTaskDelay(1 / portTICK_PERIOD_MS);
    //     }

    }
    vTaskDelete(NULL);
     
}

extern "C" void app_main(void)
{
    xTaskCreate(topstepperTask, "topstepperTask", 10000, NULL, 5, NULL);
}