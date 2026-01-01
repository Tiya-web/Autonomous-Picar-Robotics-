import time
import cv2
import numpy as np
import argparse
import math
from picar import PiCar

# Argument parsing + initializing car
parser = argparse.ArgumentParser(description="Track a blue object with a servo.")
parser.add_argument("--tim", type=float, default=10, help="Time to run the tracking program (seconds).")
parser.add_argument("--delay", type=float, default=0.2, help="Time between image captures (seconds).")
parser.add_argument("--debug", action="store_true", default=False, help="Enable debug mode.")
parser.add_argument('--mock_car', action='store_true', help='If not present, run on car, otherwise mock hardware.')
parser.add_argument("--delta", type=float, default=0.5, help="Gain value for fine-tuning servo adjustments")
parser.add_argument("--log_file", type=str, default="log.txt", help="File to open for data")

args = parser.parse_args()

car = PiCar(mock_car=args.mock_car, threaded=True)

# Constants + initial settings
ratio = (10 - (-10)) / 180  # Ratio converts 180-degree range into duty cycle range from -10 to 1
angle = 180
dutyCycle = 0
delta = args.delta          # Define delta from argument
car.set_steer_servo(0)

# TEMPORARY
with open(args.log_file, "a") as log_file:
    log_file.write("Time(s), DutyCycle, Angle\n")

    # Start tracking
    i = 0                       # Incrementation tracker
    start_time = time.time()
    cur_time = start_time

    while (start_time + args.tim > cur_time):
        cur_time = time.time()
        if cur_time > start_time + i * args.delay:
            if args.mock_car:
                array = np.zeros((480, 640, 3), dtype=np.uint8)  # simulate a blank frame
            else:
                array = car.get_image()
                
            if array is not None:

                # Convert image to BGR format
                arrayBGR = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
                    
                # Convert to HSV and create mask
                hsv = cv2.cvtColor(arrayBGR, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, (100, 150, 150), (130, 255, 255))
                mask_blur = cv2.blur(mask, (5, 5))
                thresh = cv2.threshold(mask_blur, 200, 255, cv2.THRESH_BINARY)[1]
                 
                # Calculate moments
                M = cv2.moments(thresh)

                # If no object is detected, reset angle
                if M["m00"] == 0:
                    angle = 360
                    print('DutyCycle: 0 ', 'Angle: ', angle)
                    if args.debug:
                       print("Object not detected.")
                else:
                    # Calculate the center of mass
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Calculate angle and update duty cycle
                    x = array.shape[0]
                    y = array.shape[1]
                    angle = math.atan((cX - x / 2) / (y - cY))
                    angle = math.degrees(angle)
                    dutyCycle = (dutyCycle + delta * angle * ratio)
                    dutyCycle = max(-10, min(10, dutyCycle))
                    print('DutyCycle: ', dutyCycle, 'Angle: ', angle)    

                    # Set servo angle
                    car.set_steer_servo(dutyCycle)
                    
                current_time = time.time() - start_time
                log_file.write(f"{current_time:.2f}, {dutyCycle:.2f}, {angle:.2f}\n")

                time.sleep(0.1)

                # Increment counter
                i += 1



# Stop the car
car.stop()
