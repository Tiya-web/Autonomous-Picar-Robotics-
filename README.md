# Template_Module_10

This module is for the final project.

## PiCar Configuration

Previously, there were CONFIG.txt files for each of the cars for setting the servo-motor ranges.The hardware
is swapped too frequently to maintain these configuration files for each of the cars. Please run the 
configure_servos.py script in the repository to create your own configuration file for that particular PiCar.

## It includes
- Several example data files, good and one bad
- configure_servos.py - This program is used for configuring each of the servo-motors ranges and "center".
  (Note: low-end means left (swivel, steer), down (nod) and high-end means right (swivel, steer), up (nod))
- check_servos.py - This program is good for checking the span of the servos for a given configuration file
- script1.py (and 2,3,4) - These programs are from Module 9 and can be used on the real PiCar for this module

## Deliverables
- objective1.py # Data collection for speed calculation
- objective2.py # Goes to the wall without hitting it
- objective3.py # Movement with keyboard steering control
- objective4.py # Find the object
- objective5.py # If you did it
- car_noload_5rps.txt # car on table running at 5 rps target
- manual_car_[RPSVAL]rps.txt   # car on floor going at RPSVAL target (4, 5, or 6 rps), control the speed while steering with the keyboard keys
- car_[RPSVAL]rps.txt   # car on floor going at RPSVAL target (4, 5, or 6 rps), it will have starting and stopping values
 data_9c.txt # repeat of module9c.py on the real PiCar
- plot_9c.png # repeat of plotting data from module9c.py on the real PiCar
- fft_9c.png # repeat of 

## NOTE:
For data files, data_example.txt and data_example2.txt are examples of acceptable data files.  
They include the sampling time and note that the second file includes more data than what was 
being asked for, which is ok, as long as the required elements are present.

data_example_bad.txt has two problems.  
- it is missing the sampling period to start the first line
- note that the time samples are not equally spaced

Also, do not include extra lines without data.  So if you have 10,000 element arrays for example,
but only 2000 lines that contain actual values, do not inculde 8000 lines of 0's.
