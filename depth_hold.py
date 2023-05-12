import ms5837
import time
from Adafruit_PCA9685 import PCA9685

# Initialize PCA9685 object
freq = 50
pwm = PCA9685()
pwm.set_pwm_freq(freq) # Set PWM frequency to 50Hz

sensor = ms5837.MS5837_30BA()

# Set PID values
Kp = 0.1
Ki = 0.01
Kd = 0.01

error_sum = 0
# Set maximum duty cycle
base_speed = 1500
max_speed = 100 # Limit to 50% of maximum speed

t1_channel = 0
t2_channel = 1
t3_channel = 2
t4_channel = 3

sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
target_depth = 5

def writeSpeed(speed = 0):
        wr_sp = base_speed + speed
        period = (1/freq)*1000000 
        pwm_duty_cycle = int((wr_sp/period)*4096)
        pwm.set_pwm(t1_channel,0,pwm_duty_cycle)
        pwm.set_pwm(t2_channel,0,pwm_duty_cycle)
        pwm.set_pwm(t3_channel,0,pwm_duty_cycle)
        pwm.set_pwm(t4_channel,0,pwm_duty_cycle)


if not sensor.init():
        print("Sensor could not be initialized")
        exit(1)
while True:
        if sensor.read():
                depth = sensor.depth()
                error = target_depth - depth
                error_sum += error # Add error to sum for integral term
                error_delta = error - last_error # Calculate change in error for derivative term
                last_error = error # Update last error
                output = Kp * error + Ki * error_sum + Kd * error_delta
                output = max(min(output, max_speed), 0)

        else:
                print("Sensor read failed!")
                exit(1)