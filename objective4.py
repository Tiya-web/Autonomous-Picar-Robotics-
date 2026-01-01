from picar import PiCar
import time
from time import sleep
import cv2
import numpy as np
import argparse
import math

# Argument parser setup
parser = argparse.ArgumentParser(description="Objective 4")
parser.add_argument("--rps", type=float, default=4.0)
parser.add_argument("--adSample", type=float, default=5)  # in ms
parser.add_argument("--speedCalc", type=float, default=250)  # in ms
parser.add_argument("--distDelay", type=float, default=5)  # in ms
parser.add_argument("--adDelay", type=float, default=0)
parser.add_argument("--motorDelay", type=float, default=1)
parser.add_argument('--mock_car', action='store_true', help='Use mock hardware')
parser.add_argument("--tim", type=float, default=10, help="Time to run the tracking program (seconds).")
parser.add_argument("--cameraDelay", type=float, default=0.5, help="Time between image captures (seconds).")
parser.add_argument("--debug", action="store_true", default=False, help="Enable debug mode.")
parser.add_argument("--imgsave", action="store_true", default=False, help="Save images.")
parser.add_argument("--delta", type=float, default=1, help="Gain value for fine-tuning servo adjustments")
parser.add_argument("--Kp", type=float, default=0)
parser.add_argument("--Ki", type=float, default=0)
parser.add_argument("--Kd", type=float, default=0.0)
args = parser.parse_args()

# Initialize the car object
car = PiCar(mock_car=args.mock_car, threaded=True)

# Constants
pwm = 0
ad_interval = args.adSample / 1000
calc_interval = args.speedCalc / 1000
dist_interval = args.distDelay / 1000
open_loop_gain = args.rps / 0.08
ratio = 20 / 180
angle = 180
dutyCycle = 0
delta = args.delta
car.set_swivel_servo(0)
car.set_steer_servo(0)
car.set_nod_servo(-5)
car.set_motor(pwm)

# Arrays and variables
MAXSIZE = 4000
time_arr = [0] * MAXSIZE
reading = [0] * MAXSIZE
diff = [0] * MAXSIZE
trans = [0] * MAXSIZE
error = [0] * MAXSIZE
RPSs = [0] * MAXSIZE

i1 = 0
i2 = 1
i3 = 0
i = 0
j = 0
count = 0
up = False
rps = 0.0
avg_rps = 0.0
latest_rps = 0.0
rps_count = 0
sum_error = 0.0
last = 0.0
start = 0.0

# Start tracking
start_time = time.time()
cur_time = start_time

while (start_time + args.tim > cur_time):
    cur_time = time.time()
    now = cur_time - start_time

    if start_time + i * dist_interval < cur_time:
        dist = car.read_distance()

        if dist is not None:
            if dist < 40:
                car.set_motor(20, forward = False)
                car.set_motor(0)
            elif dist < 50:
                car.set_motor(5)
            elif dist < 60:
                car.set_motor(10)
            elif dist < 120:
                car.set_motor(15)
            elif dist < 150:
                car.set_motor(20)
            elif dist < 175:
                car.set_motor(50)
            else:
                if now >= i1 * ad_interval:
                    val = car.adc.read_adc(0)
                    time_arr[i1 % MAXSIZE] = now
                    reading[i1 % MAXSIZE] = val
                    RPSs[i1 % MAXSIZE] = rps

                    if i1 == 0:
                        diff[0] = 0
                    else:
                        diff[(i1 - 1) % MAXSIZE] = reading[i1 % MAXSIZE] - reading[(i1 - 1) % MAXSIZE]

                    if i1 % MAXSIZE >= 100:
                        ratio = max(diff[i1 % MAXSIZE - 100:i1 % MAXSIZE]) * 0.3
                    else:
                        ratio1 = max(diff[MAXSIZE - (100 - i1 % MAXSIZE):])
                        if i1 % MAXSIZE != 0:
                            ratio2 = max(diff[:i1 % MAXSIZE])
                            ratio = max(ratio1, ratio2) * 0.35
                        else:
                            ratio = ratio1 * 0.35

                    dat = diff[(i1 - 1) % MAXSIZE]
                    if dat > ratio and up:
                        trans[i1 % MAXSIZE] = 1
                        up = False
                    elif dat < -ratio and not up:
                        trans[i1 % MAXSIZE] = -1
                        up = True
                    else:
                        trans[i1 % MAXSIZE] = 0
                        
                    i1 += 1

                if now >= i2 * calc_interval:
                    count = 0
                    j = i1
                    while count < 5 and j > 0:
                        if trans[j] == 1 or trans[j] == -1:
                            if count == 0:
                                start = time_arr[j]
                            elif count == 4:
                                last = time_arr[j]
                            count += 1
                        j -= 1

                    if count == 5 and start != last:
                        rps = 1 / (start - last)
                        avg_rps += rps
                        rps_count += 1
                        latest_rps = rps

                    err = args.rps - rps
                    error[i2 % MAXSIZE] = err
                    sum_error += err
                    delta_error = err - error[(i2 - 1) % MAXSIZE]
                    pwm = open_loop_gain + args.Kp * err + args.Ki * sum_error + args.Kd * delta_error
                    pwm = max(0, min(100, pwm))
                    car.set_motor(pwm)

                    i2 += 1

                    if args.debug:
                        print("RPS: {:.3f}, Error: {:.3f}, PWM: {:.2f}".format(rps, err, pwm))

            print(f"Distance: {dist}")
            i += 1

    if cur_time > start_time + i3 * args.cameraDelay:
        array = car.get_image()

        if array is not None:
            arrayBGR = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
            hsv = cv2.cvtColor(arrayBGR, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (100, 80, 120), (120, 255, 255))
            mask_blur = cv2.blur(mask, (5, 5))
            thresh = cv2.threshold(mask_blur, 100, 255, cv2.THRESH_BINARY)[1]
            M = cv2.moments(thresh)

            if args.imgsave:
                cv2.imwrite('mask.png', mask)
                cv2.imwrite('blur.png', mask_blur)
                cv2.imwrite('thresh.png', thresh)
                cv2.imwrite('array.png', arrayBGR)

            if M["m00"] == 0:
                angle = 360
                print('DutyCycle: 0 ', 'Angle: ', angle)
                if args.debug:
                    print("Object not found")
            else:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                if args.imgsave:
                    arrayCOM = cv2.circle(arrayBGR, (cX, cY), 5, (0, 0, 255), 2)
                    cv2.imwrite('arrayCOM.png', arrayCOM)

                # Calculate angle and update duty cycle
                height = array.shape[0]
                width = array.shape[1]
                angle = math.atan((cX - width / 2) / (height - cY))
                angle = math.degrees(angle)
                dutyCycle = dutyCycle + delta * angle * ratio
                dutyCycle = max(-10, min(10, dutyCycle))
                print('DutyCycle: ', dutyCycle, 'Angle: ', angle)

                # Set servo angle
                car.set_swivel_servo(dutyCycle)
                car.set_steer_servo(dutyCycle)

# Logging data
with open("car_5.0_rps.txt", "w") as data:
    for i in range(len(time_arr)):
        data.write(f'{time_arr[i]:.4f}\t{reading[i]}\t{RPSs[i]:.3f}\t{trans[i]}\n')
        
car.stop()
