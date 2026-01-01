# import PiCar class
from picar import PiCar
import time
import argparse

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--mock_car', action='store_true', 
                    help='If not present, run on car, otherwise mock hardware')
args = parser.parse_args()

# initialize PiCar:
# mock_car specifies whether you are using real hardware or the RPi
# pins is optional but can be used to override default pins for testing
# config_name is used to specify an alternate servo configuration filename
car = PiCar(mock_car=args.mock_car)

# turn on the DC motor- duty_cycle ranges 0-100, forward is optional but is either True (forward)
# or False (backward)
car.set_motor(100)
time.sleep(1)

# turn DC motor to 50% duty cycle going backwards
car.set_motor(50, forward=False)
time.sleep(1)

car.set_motor(0)

# set the servo positions
# range for servo functions is -10 (down/left) to 10 (up/right) with 0 being center
#car.set_nod_servo(-10)
#time.sleep(1)
car.set_swivel_servo(0)
time.sleep(1)
#car.set_steer_servo(10)
#time.sleep(1)

# read ultrasonic distance
dist = car.read_distance()
print(f'distance: {dist:.2f} cm')

# the PiCar exposes an ADC which can be interfaced with exactly like you would with the MCP3008
p0 = car.adc.read_adc(0)
print(f'ad_0: {p0}')

## 1-xaccel, 2-yaccel, 3-zaccel - in g
## 4-xgyro, 5-ygyro, 6-zgyro - in deg/sec
yaccel = car.MPU_Read(2)   # read the y acceleration
print(f'y acceleration: {yaccel:.2f} g')   # print the y acceleration
