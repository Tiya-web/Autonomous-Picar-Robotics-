import argparse
import time
from picar import PiCar

# Argument parsing + initializing picar
parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--mock_car', action='store_true', help='Program to run based on output of sensor')
parser.add_argument('--tim', type=float, default=10, help='Time for program run.')
parser.add_argument('--ADdelay', type=float, default=1.0, help='Delay between A/D readings')
parser.add_argument('--debug', action='store_true', help='Print debug info')
args = parser.parse_args()

picar = PiCar(mock_car=args.mock_car, threaded=False)

start_time = time.time()
cur_time = start_time
delay = args.ADdelay

i = 0
while cur_time - start_time < args.tim:
    cur_time = time.time()

    if start_time + i * delay < cur_time:
        distance = picar.read_distance()

        # Determine duty cycle based on distance
        if distance < 10 or distance >= 50:
            duty = 0
        else:
            duty = ((int((distance - 10) / 5) + 1) * 10)  # increases in 10% steps

        picar.set_motor(duty)

        # Print debug information if enabled
        if args.debug:
            print(f"Time: {cur_time - start_time:.2f}s | Distance: {distance:.2f} cm | PWM: {duty}%")
        else:
            print(f"Distance: {distance:.2f} cm")

        i += 1
        time.sleep(0.001)  

picar.stop()
