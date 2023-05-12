import ms5837
sensor = ms5837.MS5837_30BA()

# Set PID values
Kp = 0.1
Ki = 0.01
Kd = 0.01

error_sum = 0
# Set maximum duty cycle
max_speed = 100 # Limit to 50% of maximum speed

sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
target_depth = 5
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