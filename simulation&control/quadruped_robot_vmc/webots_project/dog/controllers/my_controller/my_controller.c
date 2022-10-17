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
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  WbDeviceTag motor2 = wb_robot_get_device("motor2");
  WbDeviceTag motor1 = wb_robot_get_device("motor1");
  WbDeviceTag motor2_pos_sensor = wb_robot_get_device("motor2 position sensor");
  WbDeviceTag motor1_pos_sensor = wb_robot_get_device("motor1 position sensor");
  wb_position_sensor_enable(motor1_pos_sensor,TIME_STEP);
  wb_position_sensor_enable(motor2_pos_sensor,TIME_STEP);
  
  
   float th1,th2=0;
   float hu=0.2;
   float hl=0.2;
   float x_last=0;
   float z_last=0.3;
   float v_x=0.0;
   float v_z=0.0;
   float K_x=2000;
   float D_x=500;
   float K_z=2000;
   float D_z=500;
   float tao1=0.0;
   float tao2=0.0;
   float f_x=0.0;
   float f_z=0.0;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   // J =
 
// [  hl*cos(th1 + th2) + hu*cos(th1),  hl*cos(th1 + th2)]
// [- hl*sin(th1 + th2) - hu*sin(th1), -hl*sin(th1 + th2)]
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

     th1=wb_position_sensor_get_value(motor1_pos_sensor);
     th2=wb_position_sensor_get_value(motor2_pos_sensor);
     float   x=hu*sin(th1)+hl*sin(th1+th2);
     float   z=hu*cos(th1)+hl*cos(th1+th2);
     v_x = (x-x_last)/0.064;
     v_z = (z-z_last)/0.064;
     f_x = K_x*(0-x) + D_x*(0-v_x);
     f_z = K_z*(0.2-z)+D_z*(0-v_z);
     tao1= (hl*cos(th1 + th2) + hu*cos(th1))*f_x+(- hl*sin(th1 + th2) - hu*sin(th1))*f_z;
     tao2= hl*cos(th1 + th2)*f_x+(-hl*sin(th1 + th2))*f_z;
         wb_motor_set_torque(motor2, tao2);
         wb_motor_set_torque(motor1, tao1);
        printf("th1: %f th2:%f \n",th1,th2);
        printf("x: %f z:%f \n",x,z);
        x_last = x;
        z_last = z;

  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
  
}
