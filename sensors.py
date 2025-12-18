import sys
import time

try:
    from navio2.mpu9250 import MPU9250
    from navio2.ms5611 import MS5611
    from navio2.leds import Led
    import navio2.util as util

    # Ensure the script is run with root privileges
    util.check_apm()

    # Initialize LED to indicate script status
    led_controller = Led()
    led_controller.setColor('Yellow')

    # Initialize IMU (MPU9250)
    imu_sensor = MPU9250()
    imu_sensor.initialize()

    # Initialize Barometer (MS5611)
    barometer = MS5611()
    barometer.initialize()

    # Set LED to green to indicate sensors are ready
    led_controller.setColor('Green')

    # Get IMU data
    accel_data, gyro_data, mag_data = imu_sensor.getMotion9()

    # Get Barometer data
    barometer.refreshPressure()
    barometer.refreshTemperature()
    time.sleep(0.01)
    barometer.readPressure()
    barometer.readTemperature()
    pressure = barometer.returnPressure()
    temperature = barometer.returnTemperature()

    # Print sensor readings once
    print(f"Accelerometer: {accel_data}")
    print(f"Gyroscope: {gyro_data}")
    print(f"Magnetometer: {mag_data}")
    print(f"Pressure: {pressure} Pa")
    print(f"Temperature: {temperature} C")

except ImportError as e:
    print(f"Import Error: {e}", file=sys.stderr)
    sys.exit(1)
except Exception as e:
    print(f"An error occurred: {e}", file=sys.stderr)
    sys.exit(1)
