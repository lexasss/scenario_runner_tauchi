#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
# Copyright (c) 2023 Tampere University
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Vehicle-Passes-By scenario:

The scenario realizes a driving behavior, in which the user-controlled ego vehicle runs in autonomous mode, 
and another runs behind it for some time. The this car accelerates and passes by the ego-var with a known speed.
"""

import carla
import copy
import math
import random
import subprocess
import sys

from typing import cast, List, Tuple, Optional

from py_trees.composites import (Parallel, Sequence)
from py_trees.common import ParallelPolicy

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      Idle,
                                                                      ChangeAutoPilot,
                                                                      calculate_distance)
from srunner.scenariomanager.scenarioatomics.custom_behaviors import (DebugPrint,
                                                                      Log,
                                                                      WaitForEvent,
                                                                      StopVehicleImmediately,
                                                                      ApproachFromBehind,
                                                                      EMirrorsScoresClient)
from srunner.scenariomanager.scenarioatomics.custom_trigger_conditions import DistractorSpawner
from srunner.scenarios.basic_scenario import BasicScenario

DISTANCES = [7, 15, 25, 40]
DISTANCE_REPETITIONS = 5

# pauses in seconds
PAUSE_MIN = 20
PAUSE_MAX = 30
PAUSE_INTERVIEW = 12

# velocities in m/s
VELOCITY_MAIN = 25.0
VELOCITY_APPROACHING = 3.0

LOG_SERVER_IP = '192.168.1.141'

HIDDEN_LOCATION_START = carla.Location(100, -169, 1)
HIDDEN_DIST_BETWEEN_CARS = 6

DISTRACTOR = "trafficcone01"
DISTRACTOR_DISTANCE_TO_APPEAR = 200     # meters from the ego-car
DISTRACTOR_LIFE_DURATION = 7            # seconds since appearance
DISTRACTOR_LOCATIONS = [
    carla.Location(-294.2, 19.1, 0.9),
    carla.Location(-513.8, 182.9, 0.1),
    carla.Location(-414.9, 410.5, 0.1),
    carla.Location(-109.7, 415.8, 0.1),
    carla.Location(2.9, 192.2, 0.1),
    carla.Location(1.6, -91.0, 0.1),
    carla.Location(136.8, -363.2, 0.1),
    carla.Location(386.8, -283.5, 0.1),
    carla.Location(358.8, 20.9, 0.3),
    carla.Location(56.2, 6.8, 11.1),

    carla.Location(-282.1, 6.1, 1.3),
    carla.Location(-489.5, 35.2, 0.1),
    carla.Location(-460.0, 375.8, 0.1),
    carla.Location(-195.9, 422.0, 0.1),
    carla.Location(1.7, 242.6, 0.1),
    carla.Location(14.8, -39.0, 0.1),
    carla.Location(30.8, -287.2, 0.1),
    carla.Location(315.2, -362.6, 0.1),
    carla.Location(382.0, -120.1, 0.1),
    carla.Location(156.2, 8.0, 9.2),

    carla.Location(-276.9, 19.1, 1.4),
    carla.Location(-501.2, 199.6, 0.1),
    carla.Location(-365.3, 429.9, 0.1),
    carla.Location(-94.0, 392.8, 0.1),
    carla.Location(15.8, 179.7, 0.1),
    carla.Location(1.3, -149.3, 0.1),
    carla.Location(179.7, -365.0, 0.1),
    carla.Location(383.0, -221.1, 0.1),
    carla.Location(375.6, 10.7, 0.1),
    carla.Location(71.0, 19.9, 11.1),
]

OPPONENT_CAR_TM_PARAMS = {
    "auto_lane_change": False,
    "distance_between_vehicles": 25,
    "ignore_vehicles_percentage": 0,
    "ignore_lights_percentage": 100,
    "ignore_signs_percentage": 100
}

# to be enabled in dev mode
# DISTANCES = [40]
# DISTANCE_REPETITIONS = 1
# LOG_SERVER_IP = '192.168.1.183'
LOG_SERVER_IP = None

class EMirrors(BasicScenario):

    """
    There are ~30 vehicles driving in the same direction on the highway + the ego-car. 
    At some moment one car behind is selected to be an Opponent.
    Driving is paused when the Opponen approaches te ego car close enough.
    Then the 
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, params=""):
        """
        Setup all relevant parameters and create scenario
        """

        print(f"E-MIRRORS: params = {params}")

        if params == "2":
            # subprocess.Popen(["D:\\dotnet-window-capture\\Source\\WinRT.GraphicsCapture\\bin\\Debug\\WinRT.GraphicsCapture.exe", "MultiWindow"])
            subprocess.Popen([sys.executable, "./srunner/scenarios/emirror_overlay/__main__.py"])

        self.timeout = timeout

        self._world = world
        self._map = CarlaDataProvider.get_map()

        self._ego_car: carla.Vehicle = ego_vehicles[0]
        self._other_cars: List[Tuple[carla.Vehicle, carla.Transform]] = []

        self._distances = self._create_distances()

        Log.open()

        if LOG_SERVER_IP is not None:
            EMirrorsScoresClient.open(LOG_SERVER_IP)

        super(EMirrors, self).__init__("EMirrors",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)
        
    # override
    def _initialize_actors(self, config):

        self._ego_car = self.ego_vehicles[0]

        # add actors from JSON file
        for actor in config.other_actors:
            
            actor_ = CarlaDataProvider.request_new_actor(
                actor.model,
                actor.transform,
                color="0,0,0",
                autopilot=True)
            vehicle = cast(carla.Vehicle, actor_)
            vehicle.set_simulate_physics(enabled=False)
            self.other_actors.append(vehicle)

            self._other_cars.append((vehicle, actor.transform))

    # override
    def _create_behavior(self):

        root = Parallel("Root", policy=ParallelPolicy.SUCCESS_ON_ONE)

        trials = Sequence("Trials")

        trials.add_child(EMirrorsScoresClient("start"))
        trials.add_child(ChangeAutoPilot(
            self._ego_car,
            activate=True,
            name="Ego_AutoPilot",
            parameters={
                "auto_lane_change": False
            }))

        trial_index = 1
        for distance in self._distances:

            trial = Parallel(f"Trial{trial_index}", policy=ParallelPolicy.SUCCESS_ON_ONE)

            # Ego car
            ego_car_sequence = Sequence(f"T{trial_index}_Ego")
            
            run_and_wait_car_approaching = Parallel("RunAndWaitCarApproaching", policy=ParallelPolicy.SUCCESS_ON_ONE)
            run_and_wait_car_approaching.add_child(self._follow_waypoints(
                self._ego_car, "Ego",
                VELOCITY_MAIN))
            pause = PAUSE_MIN + random.random() * (PAUSE_MAX - PAUSE_MIN)
            run_and_wait_car_approaching.add_child(ApproachFromBehind(
                self._ego_car,
                [car for (car, _) in self._other_cars],
                distance,
                pause,
                self._world,
                trial_index
            ))
            
            ego_car_sequence.add_child(DebugPrint(self._ego_car, f"T{trial_index}_EGO: Trial started"))
            ego_car_sequence.add_child(DebugPrint(self._ego_car, f"T{trial_index}_EGO: distance = {distance} m, pause = {pause:.1f} sec"))
            ego_car_sequence.add_child(run_and_wait_car_approaching)
            ego_car_sequence.add_child(WaitForEvent(trial_index, 0))    # sets the event with ID = trial_index
            ego_car_sequence.add_child(StopVehicleImmediately(self._ego_car))
            ego_car_sequence.add_child(DebugPrint(self._ego_car, f"T{trial_index}_EGO: A car approached. Pause for the interview"))
            ego_car_sequence.add_child(Log("trial", "stage", "interview"))
            ego_car_sequence.add_child(EMirrorsScoresClient("pause"))
            ego_car_sequence.add_child(self._follow_waypoints(
                self._ego_car, f"T{trial_index}_Ego",
                velocity=0,
                duration=PAUSE_INTERVIEW,
                event_id=trial_index + 1000))
            ego_car_sequence.add_child(DebugPrint(self._ego_car, f"T{trial_index}_EGO: Trial finished"))
            ego_car_sequence.add_child(EMirrorsScoresClient("continue"))
            
            trial.add_child(ego_car_sequence)

            # Other cars
            car_index = 1
            for other_car, other_car_transform in self._other_cars:
                other_car_sequence = Sequence(f"T{trial_index}_OtherCar{car_index}")
                other_car_sequence.add_child(ActorTransformSetter(
                    other_car,
                    other_car_transform,
                    name=f"T{trial_index}_OtherCar{car_index}_Placement"))
                other_car_sequence.add_child(self._follow_waypoints(
                    other_car, f"T{trial_index}_OtherCar{car_index}",
                    VELOCITY_MAIN + VELOCITY_APPROACHING,
                    event_id=trial_index))
                other_car_sequence.add_child(StopVehicleImmediately(other_car))
                other_car_sequence.add_child(ActorTransformSetter(
                    other_car,
                    carla.Transform(carla.Location(
                        HIDDEN_LOCATION_START.x + car_index * HIDDEN_DIST_BETWEEN_CARS,
                        HIDDEN_LOCATION_START.y,
                        HIDDEN_LOCATION_START.z)),
                    name=f"T{trial_index}_OtherCar{car_index}_Hide"))
                other_car_sequence.add_child(self._follow_waypoints(
                    other_car, f"T{trial_index}_OtherCar{car_index}",
                    velocity=0,
                    event_id=trial_index + 1000))

                trial.add_child(other_car_sequence)
                car_index += 1
            
            trials.add_child(Log("trial", "start", trial_index, distance, f"{pause:.1f}"))
            trials.add_child(EMirrorsScoresClient(f"trial\t{trial_index}\tstart\t{distance}\t{pause:.1f}"))
            trials.add_child(trial)
            trials.add_child(EMirrorsScoresClient(f"trial\t{trial_index}\tend"))
            trials.add_child(Log("trial", "stop", trial_index))

            trial_index += 1
        
        trials.add_child(EMirrorsScoresClient("stop"))
        trials.add_child(Idle(1.0))
        trials.add_child(DebugPrint(self._ego_car, "Finished"))
        trials.add_child(Idle(1.0))
        
        root.add_child(trials)

        distractor_sequence = Sequence("Distractor")
        distractor_sequence.add_child(DistractorSpawner(
            self._ego_car,
            DISTRACTOR,
            DISTRACTOR_LOCATIONS,
            distance=DISTRACTOR_DISTANCE_TO_APPEAR,
            duration=DISTRACTOR_LIFE_DURATION))

        root.add_child(distractor_sequence)
        
        return root

    # override
    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used in parallel behavior tree.
        """

        return []

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
        Log.close()
        EMirrorsScoresClient.close()


    # Internal
    
    def _follow_waypoints(self, vehicle, name, velocity, duration=None, event_id=None):
        """
        Follows waypoints
        """
        parallel = Parallel(f"{name}_DrivingUtilTimeout",
                            policy=ParallelPolicy.SUCCESS_ON_ONE)

        parallel.add_child(WaypointFollower(
            vehicle,
            velocity,
            avoid_collision=True,
            name=f"{name}_LaneFollower"))

        if duration is not None:
            if event_id is not None:
                # -- Drive for a certain period and set the global event at the end
                parallel.add_child(WaitForEvent(event_id, duration, name=f"{name}_UntilTimeoutThenSetEvent"))
            else:
                # -- Drive for a certain period
                parallel.add_child(Idle(duration, name=f"{name}_UntilTimeout"))
        else:
            if event_id is not None:
                # -- Until the global event is set
                parallel.add_child(WaitForEvent(event_id, name=f"{name}_UntilEvent"))
            else:
                # -- Until ego/opponent exist
                parallel.add_child(Idle(name=f"{name}_UntilFinished"))

        return parallel

    def _create_distances(self):
        distances: List[float] = []
        for i in range(DISTANCE_REPETITIONS):
            randomized_distances = copy.copy(DISTANCES)
            random.shuffle(randomized_distances)
            distances.extend(randomized_distances)

        print("E-MIRRORS: Distances: " + " ".join([f"{d:.1f}" for d in distances]))
        return distances