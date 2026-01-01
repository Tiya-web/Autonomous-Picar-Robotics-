from time import sleep
import argparse
import time
from picar import PiCar

parser = argparse.ArgumentParser(description="Real time speed and control.")
parser.add_argument("--rps", type=float, default=5, help="Target RPS.")
parser.add_argument("--tim", type=float, default=10, help="Time for program run.")
parser.add_argument("--sample_delay", type=float, default=0.005, help="Time between AD samples.")
parser.add_argument("--calc_delay", type=float, default=0.15, help="Time between speed calculations.")
parser.add_argument("--start_delay", type=float, default=0.25, help="Delay before starting control.")
parser.add_argument("--Kp", type=float, default=2, help="Proportional gain.")
parser.add_argument("--Kd", type=float, default=0, help="Derivative gain.")
parser.add_argument("--Ki", type=float, default=1.6, help="Integral gain.")
parser.add_argument('--mock_car', action='store_true', help='Use mock hardware')
parser.add_argument("--debug", action="store_true", default=False, help="Enable debug mode.")
args = parser.parse_args()

car = PiCar(mock_car=args.mock_car, threaded=True)
car.set_swivel_servo(0)

goal_rps = args.rps
pwm_desired = (args.rps + 4.1316) / 0.109
open_loop_gain = pwm_desired
print('PWM Desired:', pwm_desired)

if args.mock_car:
    pwm_desired = (args.rps - 2) / 0.0627
    open_loop_gain = pwm_desired

car.set_motor(pwm_desired)

MAXSIZE = 4000
time_arr = [0] * MAXSIZE
reading = [0] * MAXSIZE
diff = [0] * MAXSIZE
trans = [0] * MAXSIZE
error = [0] * MAXSIZE

i1 = 0
i2 = 1
i = 0
j = 0
count = 0
up = False
rps = 0
sum = 0
avg = 0
counter = 0
last = 0
start = 0

sleep(args.start_delay)
start_time = time.time()
cur_time = time.time()

dc = 0
going = True
car.set_steer_servo(0)

with open('manual_car_xxrps.txt', 'w') as data:
    while going:
        key = car.get_keyin()
        cur_time = time.time()

        # Key input
        if key == 's':
            print(f'got: {key}')
            dc += 0.7
            if dc > 10:
                dc = 10
                print('Max Reached')
            elif dc < -10:
                dc = -10
                print('Min Reached')
            print('PWM of Steer:', dc)
            car.set_steer_servo(dc)

        elif key == 'd':
            print(f'got: {key}')
            dc -= 0.7
            if dc > 10:
                dc = 10
                print('Max Reached')
            elif dc < -10:
                dc = -10
                print('Min Reached')
            print('PWM:', dc)
            car.set_steer_servo(dc)

        # AD sampling
        if cur_time > start_time + i1 * args.sample_delay:
            value = car.adc.read_adc(0)
            dist = car.read_distance()

            if dist is not None:
                print('Distance', dist)
                print('')
                if dist < 38:
                    car.set_motor(0)
                    going = False

            if args.debug:
                print(value)

            time_arr[i1 % MAXSIZE] = cur_time - start_time
            reading[i1 % MAXSIZE] = value

            if i1 == 0:
                diff[0] = 0
            else:
                diff[(i1 - 1) % MAXSIZE] = reading[i1 % MAXSIZE] - reading[(i1 - 1) % MAXSIZE]

            if i1 % MAXSIZE >= 100:
                ratio = max(diff[i1 % MAXSIZE - 100:i1 % MAXSIZE]) * 0.25
            else:
                ratio1 = max(diff[MAXSIZE - (100 - i1 % MAXSIZE):MAXSIZE])
                if i1 % MAXSIZE != 0:
                    ratio2 = max(diff[:i1 % MAXSIZE])
                    ratio = max(ratio1, ratio2) * 0.25
                else:
                    ratio = ratio1 * 0.25

            if args.debug:
                print('ratio', ratio)

            dat = diff[(i1 - 1) % MAXSIZE]
            if args.debug:
                print('diff', dat)

            if dat > ratio and up:
                trans[i1 % MAXSIZE] = 1
                up = False
                if args.debug:
                    print('pos')
            elif dat < -ratio and not up:
                trans[i1 % MAXSIZE] = -1
                up = True
                if args.debug:
                    print('neg')
            else:
                trans[i1 % MAXSIZE] = 0

            i1 += 1
            data.write("{0:0.4f} {1:10.2f} {2:10.5f} {3:10.2f} {4:10.2f} \n".format(
                (cur_time - start_time),
                reading[i1 % MAXSIZE - 1],
                rps,
                reading[i1 % MAXSIZE - 1] - reading[(i1 - 2) % MAXSIZE],
                trans[i1 % MAXSIZE - 1]
            ))

        # Speed calculation and PID control
        if cur_time > start_time + i2 * args.calc_delay:
            i2 += 1
            count = 0
            j = 0
            while count < 5 and j <= 1000:
                if trans[(i1 - 1 - j) % MAXSIZE] != 0:
                    count += 1
                    if count == 1:
                        start = time_arr[j % MAXSIZE]
                    else:
                        last = time_arr[j % MAXSIZE]
                j += 1

            if args.debug:
                print('start', start)
                print('last', last)

            total = last - start
            if counter != 0 and total != 0:
                rps = 1 / total
                avg += rps
            counter += 1

            if rps < 0:
                rps = 0
            elif rps > 8:
                rps = 8

            error[i2 % MAXSIZE] = goal_rps - rps
            sum += error[i2 % MAXSIZE]
            pwm = open_loop_gain + (args.Kp * error[i2 % MAXSIZE] +
                                    args.Ki * sum +
                                    args.Kd * (error[i2 % MAXSIZE] - error[i2 % MAXSIZE - 1]))

            if pwm < 0:
                pwm = 0
                rps = 0
            elif pwm > 100:
                pwm = 100

            car.set_motor(pwm)
            print('RPS:', rps)
            print('Error:', error[i2 % MAXSIZE])
            print('PWM:', pwm)
            print('')

car.stop()
