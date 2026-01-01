
from picar import PiCar
import time
import argparse

# Argument parsing + initializing Pipicar
parser = argparse.ArgumentParser(description='Read ultrasonic and ADC values.')
parser.add_argument('--mock_car', action='store_true', help='Use mock hardware instead of real Pipicar.')
parser.add_argument('--time', type=int, default=5, help='Duration in seconds to run the sensor reading loop (default: 5)')
parser.add_argument('--debug', action='store_true')

args = parser.parse_args()

picar = PiCar(mock_car=args.mock_car)

# Loop that reads from the sensor
start_time = time.time()
while time.time() - start_time < args.time:
    distance = picar.read_distance()
    adc_val = picar.adc.read_adc(0)
    print(f"({time.strftime('%H:%M:%S')})   Distance: {distance:.2f} cm    ADC[0]: {adc_val}")
    time.sleep(1)
    
print("\nLoop stops, sensor reading complete.")
