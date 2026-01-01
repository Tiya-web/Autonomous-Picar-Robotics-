# import module
from picar import PiCar, test, configure
import argparse

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--mock_car', action='store_true', 
                    help='If not present, run on car, otherwise mock hardware')
args = parser.parse_args()

# test on actual hardware
car = PiCar(mock_car=args.mock_car)

# see current pin configuration
print(car)

# execute functionality test
# WARNING: The car will attempt to drive
test.execute_test(car)
