/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
 
// Added a new include file
#include <webots/motor.h>
 
#define TIME_STEP 64
 
#define MAX_SPEED 6.28
 
int main(int argc, char **argv) {
 wb_robot_init();
 
 // get a handler to the motors and set target position to infinity (speed control)
 WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
 WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
 wb_motor_set_position(left_motor, INFINITY);
 wb_motor_set_position(right_motor, INFINITY);
 
 // set up the motor speeds at 10% of the MAX_SPEED.
 wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
 wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);
 
 while (wb_robot_step(TIME_STEP) != -1) {
 }
 
 wb_robot_cleanup();
 
 return 0;
}