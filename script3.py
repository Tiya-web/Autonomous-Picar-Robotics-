# import module
from picar import PiCar, configure
import time
import argparse

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--mock_car', action='store_true', 
                    help='If not present, run on car, otherwise mock hardware')
args = parser.parse_args()

# test on actual hardware
car = PiCar(mock_car=args.mock_car)

car.set_swivel_servo(-10)
print(f'car swivel: {car.swivel_servo_state}')
time.sleep(1)
car.set_swivel_servo(0)
print(f'car swivel: {car.swivel_servo_state}')
time.sleep(1)
car.set_swivel_servo(10)
print(f'car swivel: {car.swivel_servo_state}')
time.sleep(1)
car.set_swivel_servo(0)
print(f'car swivel: {car.swivel_servo_state}')

# uncomment to set current pin configuration
# configure.configure_car(car)
