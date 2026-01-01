

import time
import argparse
from picar import PiCar

#argparse arguments
parser = argparse.ArgumentParser()
parser.add_argument('--tim', type=float, default=10.0)
parser.add_argument('--delay', type=float, default=0.25)
parser.add_argument('--debug', action='store_true')
parser.add_argument('--mock_car', action='store_true')
args = parser.parse_args()

# Initialize PiCar in threaded mode
car = PiCar(mock_car=args.mock_car, threaded=True)

# Initialize servo position to center
current_duty_cycle = 0
car.set_steer_servo(current_duty_cycle)
new_duty_cycle = current_duty_cycle

start_time = time.time()
mesg_time = start_time
cur_time = start_time

while (start_time + args.tim > cur_time):
    time.sleep(0.0001) 
    cur_time = time.time()

    if (mesg_time + args.delay < cur_time):
        mesg_time = cur_time
        key = car.get_keyin()

        if (key is None):
            print("No key is entered")

        if (key == 's'):
            new_duty_cycle = current_duty_cycle - 0.5
            print("key 's' is entered")
        
        if (key == 'd'):
            new_duty_cycle = current_duty_cycle + 0.5
            print("key 'd' is entered")
        
        # Ensure the servo stays within the limits of -10 to 10
        if (-10 <= new_duty_cycle <= 10):
            current_duty_cycle = new_duty_cycle

        if args.debug:
           print(f"Key: {key} | Steering PWM: {new_duty_cycle}")

car.set_steer_servo(new_duty_cycle)



#car.stop()
