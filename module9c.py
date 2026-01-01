import time
import argparse
import numpy as np
from picar import PiCar

# Argument parsing
parser = argparse.ArgumentParser(description="Motor speed tracking program with PID control using PiCar.")
parser.add_argument("--rps", type=float, default=4.0, help="Desired motor speed in RPS.")
parser.add_argument("--tim", type=float, default=4.0, help="Total time to run the program.")
parser.add_argument("--adSample", type=float, default=5, help="ADC sample rate (Hz).")
parser.add_argument("--speedCalc", type=float, default=250, help="Speed calculation rate (Hz).")
parser.add_argument("--adDelay", type=float, default=0.0, help="Time to wait before starting AD sampling.")
parser.add_argument("--motorDelay", type=float, default=1.0, help="Time to wait before turning the motor on.")
parser.add_argument("--Kp", type=float, default=0, help="Proportional gain.")
parser.add_argument("--Ki", type=float, default=0, help="Integral gain.")
parser.add_argument("--Kd", type=float, default=0, help="Derivative gain.")
parser.add_argument("--mock_car", action="store_true", help="Use mock PiCar hardware.")
parser.add_argument("--debug", action="store_true", default=False, help="Enable debug mode.")
args = parser.parse_args()

car = PiCar(mock_car=args.mock_car)


best_fit_slope = 1 / 0.0843  
initial_dc = best_fit_slope * args.rps
initial_dc = max(0, min(100, initial_dc))

# Data storage
MAXSIZE = int((args.adSample - args.adDelay) * args.tim)
AD_readings = [0] * MAXSIZE
time_arr = [0] * MAXSIZE
diff = [0] * MAXSIZE
transitions = [0] * MAXSIZE
RPS = [0] * MAXSIZE

# PID control
cur_error = 0
prev_error = 0
sum_error = 0
diff_error = 0


AD_idx = 0
rps = 0
lookingDark = True
loop1_time = loop2_time = start_time = time.time()
end_time = start_time + args.tim
photo_start = start_time + args.adDelay
motor_start = start_time + args.motorDelay
motor_started = False
ratio_RPS = int(args.adSample / args.speedCalc) * 3
movingAvgdiffs = [0] * 5

# Main loop
while time.time() < end_time:
    cur_time = time.time()

    # Start motor at the right time
    if not motor_started and cur_time >= motor_start:
        car.set_motor(initial_dc, forward=True)
        motor_started = True

    # ADC sampling
    if cur_time >= photo_start and cur_time - loop1_time >= (1 / args.adSample):
        adcVal = car.adc.read_adc(0)
        AD_readings[AD_idx] = adcVal
        time_arr[AD_idx] = cur_time - start_time
        RPS[AD_idx] = rps

        if AD_idx > 0:
            diff[AD_idx] = AD_readings[AD_idx] - AD_readings[AD_idx - 1]

            if AD_idx >= 5:
                movingAvgdiffs[AD_idx % 5] = np.mean(diff[max(0, AD_idx - 5):AD_idx])

            max_thresh = np.max(movingAvgdiffs) * 0.2
            min_thresh = np.min(movingAvgdiffs) * 0.2

            val = movingAvgdiffs[AD_idx % 5]
            if lookingDark and val > max_thresh:
                transitions[AD_idx] = 1
                lookingDark = False
            elif not lookingDark and val < min_thresh:
                transitions[AD_idx] = -1
                lookingDark = True

        loop1_time = cur_time
        AD_idx += 1

    # Speed calculation + PID control
    if cur_time >= photo_start + 0.5 and cur_time - loop2_time >= (1 / args.speedCalc):
        sum_transitions = sum(np.abs(transitions[max(0, AD_idx - ratio_RPS):AD_idx])) if AD_idx >= ratio_RPS else 0
        if sum_transitions > 0:
            time_dif = time_arr[AD_idx - 1] - time_arr[max(0, AD_idx - ratio_RPS)]
            if time_dif > 0:
                rps = sum_transitions / (time_dif * 4)
                if args.debug:
                    print(f"Time: {cur_time - start_time:.2f}s, RPS: {rps:.5f}")

        # PID update
        cur_error = args.rps - rps
        sum_error += cur_error
        diff_error = cur_error - prev_error
        prev_error = cur_error

        
        new_dc = best_fit_slope * args.rps + args.Kp * cur_error + args.Ki * sum_error + args.Kd * diff_error
        new_dc = max(0, min(100, new_dc))
        car.set_motor(new_dc, forward=True)

        loop2_time = cur_time

# Stop motor
car.set_motor(0)

# Save data
fname = "data_9c.txt"
with open(fname, 'w') as f:
    f.write("Time(s)\tADC_Value\tRPS\tTransition\n")
    for i in range(AD_idx):
        f.write(f"{time_arr[i]:.4f}\t{AD_readings[i]}\t{RPS[i]:.5f}\t{transitions[i]}\n")

print(f"Data written to {fname}")