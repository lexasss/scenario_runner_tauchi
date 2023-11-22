# This script creates car spawning locations along the trace stored in XXXX file.

# To create the trace file, uncomment all line below the 'TAUCHI trace' comment in scenario_manager.py.
# The generated spawning locations will appear in 'XXXX_vehicles.txt' file
# in a format ready to be inserted into a 'scenario_runner/srunner/data/*.json' file
# (into the section 'available_scenarios/TownXX/available_event_configurations/other_actors/front')

import math

FILENAME_INPUT = './town04_circular.txt'
FILENAME_OUTPUT = './town04_circular_vehicles.txt'
INTERVAL = 80      # meters
LANE_WIDTH = 3

input = open(FILENAME_INPUT, 'r')
output = open(FILENAME_OUTPUT, 'w')

lines = input.readlines()

current_distance = -1
sx, sy, sz = 0, 0, 0

angle = 90 * math.pi / 180

for line in lines:
    if len(line) > 0:
        p = line.split('\t')
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        
        if current_distance < 0:
            current_distance = 0
        else:
            current_distance += math.sqrt((x - sx)**2 + (y - sy)**2 + (z - sz)**2)
            if current_distance > INTERVAL:
                current_distance = 0
                
                yaw = math.atan2(y - sy, x - sx)
                
                vx = x + LANE_WIDTH * math.cos(yaw + angle)
                vy = y + LANE_WIDTH * math.sin(yaw + angle)
                vz = z + 0.5
                
                yaw *= 180 / math.pi
                
                print(f'{vx:.1f}\t{vy:.1f}\t{vz:.1f}')
                output.write('{ ' + f'"x": "{vx:.1f}", "y": "{vy:.1f}", "z": "{vz:.1f}", "yaw": "{yaw:.1f}", "model": "vehicle.tesla.model3"' + ' },\n')
                
                angle = -angle
        
        sx, sy, sz = x, y, z

input.close()
output.close()