#!/usr/bin/env python3
import argparse
import sys
import time

try:
    from navio2.mpu9250 import MPU9250
    from navio2.ms5611 import MS5611
    from navio2.leds import Led
    import navio2.util as util
except ImportError as e:
    print(f"Import Error: {e}", file=sys.stderr)
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Read Navio2 IMU and barometer data")
    parser.add_argument(
        "--once",
        action="store_true",
        help="Output a single reading then exit",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds between readings when running continuously",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        util.check_apm()

        led_controller = Led()
        led_controller.setColor("Yellow")

        imu_sensor = MPU9250()
        imu_sensor.initialize()

        barometer = MS5611()
        barometer.initialize()

        led_controller.setColor("Green")

        while True:
            accel_data, gyro_data, mag_data = imu_sensor.getMotion9()

            barometer.refreshPressure()
            time.sleep(0.01)
            barometer.readPressure()
            pressure = barometer.returnPressure()

            barometer.refreshTemperature()
            time.sleep(0.01)
            barometer.readTemperature()
            temperature = barometer.returnTemperature()

            print(f"Accelerometer: {accel_data}")
            print(f"Gyroscope: {gyro_data}")
            print(f"Magnetometer: {mag_data}")
            print(f"Pressure: {pressure} Pa")
            print(f"Temperature: {temperature} C")

            if args.once:
                break

            time.sleep(args.interval)
    except Exception as e:
        print(f"An error occurred: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
