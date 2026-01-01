import time
import cv2
import numpy as np
import argparse
import math
from picar import PiCar

# Argument parser setup
parser = argparse.ArgumentParser(description="Track a blue object with a servo.")
parser.add_argument("--tim", type=float, default=10, help="Time to run the tracking program (seconds).")
parser.add_argument("--delay", type=float, default=0.2, help="Time between image captures (seconds).")
parser.add_argument("--debug", action="store_true", default=False, help="Enable debug mode.")
parser.add_argument("--imgsave", action="store_true", default=False, help="Save images.")
parser.add_argument('--mock_car', action='store_true', help='If not present, run on car, otherwise mock hardware.')
parser.add_argument("--delta", type=float, default=1, help="Gain value for fine-tuning servo adjustments")
args = parser.parse_args()

# Initialize the car object
car = PiCar(mock_car=args.mock_car, threaded=True)

# Constants
ratio = (10 - (-10)) / 180  # in reality 20/180 
angle = 180
dutyCycle = 0
delta = args.delta  # Define delta from argument
car.set_swivel_servo(0)
car.set_steer_servo(0)
car.set_nod_servo(5)

# Start tracking
i = 0
start_time = time.time()
cur_time = start_time

while (start_time + args.tim > cur_time):
    cur_time = time.time()
    if cur_time > start_time + i * args.delay:
        array = car.get_image()

        # Determining the speed of the car from program 9b
        if start_time + i * args.delay < cur_time:

            dist = car.read_distance()

            if dist < 20:
                car.set_motor(5, forward = False)
                car.set_motor(0)
            elif dist < 30:
                car.set_motor(5)
            elif dist < 40:
                car.set_motor(10)
            elif dist < 50:
                car.set_motor(20)
            elif dist < 60:
                car.set_motor(30)
            elif dist < 70:
                car.set_motor(40)
            else:
                car.set_motor(75)

            print(f"Distance: {dist}")
            i += 1

            if array is not None:
                # Convert image to BGR format
                arrayBGR = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)

                # Convert to HSV and create mask
                hsv = cv2.cvtColor(arrayBGR, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, (100, 80, 120), (120, 255, 255))
                mask_blur = cv2.blur(mask, (5, 5))
                thresh = cv2.threshold(mask_blur, 100, 255, cv2.THRESH_BINARY)[1]

                # Calculate moments
                M = cv2.moments(thresh)

                # Save images if enabled
                if args.imgsave:
                    cv2.imwrite('mask.png', mask)
                    cv2.imwrite('blur.png', mask_blur)
                    cv2.imwrite('thresh.png', thresh)
                    cv2.imwrite('array.png', arrayBGR)

                # If no object is detected, reset angle
                if M["m00"] == 0:
                    angle = 360
                    print('DutyCycle: 0 ', 'Angle: ', angle)
                    if args.debug:
                        print("Object not found")
                else:
                    # Calculate center of mass
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Save image with center of mass if enabled
                    if args.imgsave:
                        arrayCOM = cv2.circle(arrayBGR, (cX, cY), 5, (0, 0, 255), 2)
                        cv2.imwrite('arrayCOM.png', arrayCOM)

                    # Calculate angle and update duty cycle
                    # x = array.shape[0]
                    # y = array.shape[1]
                    # angle = math.atan((cX - x / 2) / (y - cY))
                    height, width = arrayBGR.shape[:2]
                    angle = math.atan((cX - height / 2) / (width - cY))  
                    angle = math.degrees(angle)
                    dutyCycle = (dutyCycle + delta * angle * ratio)
                    dutyCycle = max(-10, min(10, dutyCycle))
                    print('DutyCycle: ', dutyCycle, 'Angle: ', angle)

                    # Set servo angle
                    car.set_swivel_servo(dutyCycle)
                    car.set_steer_servo(dutyCycle)

# Stop the car
car.stop()
