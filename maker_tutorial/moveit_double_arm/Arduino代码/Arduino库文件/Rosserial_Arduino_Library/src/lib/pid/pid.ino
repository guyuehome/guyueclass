#include "PID.h"
PID pid( -255, 255, 0.05, 0.9, 0.1);
/*PID(float min_val, float max_val, float kp, float ki, float kd)
 * min_val = min output PID value
 * max_val = max output PID value
 * kp = PID - P constant
 * ki = PID - I constant
 * di = PID - D constant
 */

float setpoint = 30;
float measured_value = 0;
void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  Serial.println(pid.compute(setpoint, measured_value));
  delay(1000);
  measured_value++;
}
