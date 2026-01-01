
# A target rps of 4 was used beacuse our PiCar appeared to only reach a max rps of 4.5 with 100% PWM Duty Cycle

# Importing the nessecary libraies
from time import sleep
import argparse
import time
from picar import PiCar

# setting up the command line arguments for the program
parser = argparse.ArgumentParser(description="Objective 1")
parser.add_argument("--rps", type=float, default=4.0)
parser.add_argument("--tim", type=float, default=10)
parser.add_argument("--adSample", type=float, default=5)  # in ms
parser.add_argument("--speedCalc", type=float, default=250)  # in ms
parser.add_argument("--adDelay", type=float, default=0)
parser.add_argument("--motorDelay", type=float, default=1)
parser.add_argument("--Kp", type=float, default=2) # determined as the best value after several trials
parser.add_argument("--Ki", type=float, default=1.6) # determined as the best value after several trials
parser.add_argument("--Kd", type=float, default=0.0)
parser.add_argument("--debug", action="store_true")
parser.add_argument('--mock_car', action='store_true', help='Use mock hardware')
args = parser.parse_args()

car = PiCar(mock_car=args.mock_car)

ad_interval = args.adSample / 1000  # convert to seconds
calc_interval = args.speedCalc / 1000

open_loop_gain = (args.rps) / 0.08 
pwm = 0

if args.mock_car:
    open_loop_gain = (args.rps - 2) / 0.0827

car.set_motor(pwm)

# initializing arrays and variables
MAXSIZE = 4000
time_arr = [0] * MAXSIZE
reading = [0] * MAXSIZE
diff = [0] * MAXSIZE
trans = [0] * MAXSIZE
error = [0] * MAXSIZE
RPSs = [0] * MAXSIZE

i1 = 0
i2 = 1
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

sleep(args.motorDelay) #delays motor startup

start_time = time.time()
cur_time = start_time

# main loop with real time rps calculation and control

while cur_time - start_time < args.tim:
    cur_time = time.time()
    now = cur_time - start_time

    # ADC Sampling
    if now >= i1 * ad_interval:
        val = car.adc.read_adc(0)
        time_arr[i1 % MAXSIZE] = now
        reading[i1 % MAXSIZE] = val
        RPSs[i1 % MAXSIZE] = rps

        if i1 == 0:
            diff[0] = 0
        else:
            diff[(i1 - 1) % MAXSIZE] = reading[i1 % MAXSIZE] - reading[(i1 - 1) % MAXSIZE]

        # Transistion detection
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

    # Speed calculation and control
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
        print(i1)

        if count == 5 and start != last:
            rps = 1 / (start - last)
            avg_rps += rps
            rps_count += 1
            latest_rps = rps

        # PID Control
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

# Logging data
with open("car_noload_5rps.txt", "w") as data:
    for i in range(len(time_arr)):
        data.write(f'{time_arr[i]:.4f}\t{reading[i]}\t{RPSs[i]:.3f}\t{trans[i]}\n')

data.close()

print("Avg RPS: {:.3f}".format(avg_rps / rps_count if rps_count > 0 else 0)) 
