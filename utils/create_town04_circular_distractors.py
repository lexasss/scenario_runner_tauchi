# This script creates distractor spawning locations along the trace stored in XXXX file.
# The outcome is used in EMirror scenario

# To create the trace file, uncomment all line below the 'TAUCHI trace' comment in scenario_manager.py
# and drive a desired route. Copy the generated 'trace.txt' file to this folder.

# The generated spawning locations will appear in 'XXXX_distractors.txt' file
# in a format ready to be inserted into a 'scenario_runner/srunner/scenarios/emirrors.py' file

import math
import random

FILENAME_INPUT = './town04_circular.txt'
FILENAME_OUTPUT = './town04_circular_distractors.txt'
INTERVAL_AVG = 300    # meters
INTERVAL_SD = 50      # meters
OFFSET = 6.5          # meters

input = open(FILENAME_INPUT, 'r')
output = open(FILENAME_OUTPUT, 'w')

lines = input.readlines()

current_distance = -1
sx, sy, sz = 0, 0, 0

angle = (90 if random.random() < 0.5 else -90) * math.pi / 180
interval = INTERVAL_AVG + random.randrange(-INTERVAL_SD, INTERVAL_SD)

for line in lines:
    if len(line) > 0:
        p = line.split('\t')
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        
        if current_distance < 0:
            current_distance = 0
        else:
            current_distance += math.sqrt((x - sx)**2 + (y - sy)**2 + (z - sz)**2)
            if current_distance > interval:
                current_distance = 0
                
                yaw = math.atan2(y - sy, x - sx)
                
                vx = x + OFFSET * math.cos(yaw + angle)
                vy = y + OFFSET * math.sin(yaw + angle)
                vz = z + 0.1
                
                print(f'{vx:.1f}\t{vy:.1f}\t{vz:.1f}')
                output.write(f'    carla.Location({vx:.1f}, {vy:.1f}, {vz:.1f}),\n')

                interval = INTERVAL_AVG + random.randrange(-INTERVAL_SD, INTERVAL_SD)
                
                if random.random() < 0.5:
                    angle = -angle
        
        sx, sy, sz = x, y, z

input.close()
output.close()