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
import operator
import random

from typing import cast, List, Tuple

from py_trees.composites import (Parallel, Sequence)
from py_trees.common import ParallelPolicy

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      Idle,
                                                                      ChangeAutoPilot)
from srunner.scenariomanager.scenarioatomics.custom_behaviors import (DebugPrint,
                                                                      VehicleFollower,
                                                                      ApproachFromBehind)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle
from srunner.scenariomanager.scenarioatomics.custom_trigger_conditions import DistractorSpawner
from srunner.scenarios.basic_scenario import BasicScenario

DISTRACTOR = 'trafficcone01'
DISTRACTOR_LOCATIONS = [
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

DISTANCES = [15]

PAUSE_MIN = 30
PAUSE_MAX = 120

VELOCITY_MAIN = 25.0         # m/s
VELOCITY_PASSING_BY = 5.0    # m/s

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

        self.timeout = timeout

        self._world = world
        self._map = CarlaDataProvider.get_map()

        self._ego_car: carla.Vehicle = ego_vehicles[0]
        self._other_cars: List[Tuple[carla.Vehicle, carla.Transform]] = []

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
            
            actor_ = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            vehicle = cast(carla.Vehicle, actor_)
            vehicle.set_simulate_physics(enabled=False)
            self.other_actors.append(vehicle)

            self._other_cars.append((vehicle, actor.transform))

    # override
    def _create_behavior(self):

        root = Parallel("Root", policy=ParallelPolicy.SUCCESS_ON_ONE)
        
        trials = Sequence("Trials")
        
        for distance in DISTANCES:

            trial = Parallel("Trial", policy=ParallelPolicy.SUCCESS_ON_ONE)

            # Ego car
            ego_car_sequence = Sequence("Ego")
            ego_car_sequence.add_child(ChangeAutoPilot(
                self._ego_car,
                activate=True,
                name="Ego_AutoPilot",
                parameters={
                    "auto_lane_change": False
                }))
            
            run_and_wait_car_approaching = Parallel("RunAndWaitCarApproaching", policy=ParallelPolicy.SUCCESS_ON_ONE)
            run_and_wait_car_approaching.add_child(self._follow_waypoints(
                self._ego_car, "Ego",
                VELOCITY_MAIN))
            pause = PAUSE_MIN + random.random() * (PAUSE_MAX - PAUSE_MIN)
            run_and_wait_car_approaching.add_child(ApproachFromBehind(
                self._ego_car,
                [car for (car, _) in self._other_cars],
                distance,
                pause
            ))
            
            ego_car_sequence.add_child(run_and_wait_car_approaching)
            
            trial.add_child(ego_car_sequence)

            # Other cars
            i = 1
            for other_car, other_car_transform in self._other_cars:
                other_car_sequence = Sequence(f"OtherCar{i}")
                other_car_sequence.add_child(ActorTransformSetter(
                    other_car,
                    other_car_transform,
                    name=f"OtherCar{i}_Placement"))
                other_car_sequence.add_child(ChangeAutoPilot(
                    other_car,
                    activate=True,
                    name=f"OtherCar{i}_AutoPilot",
                    parameters={
                        "auto_lane_change": False
                    }))
                other_car_sequence.add_child(self._follow_waypoints(
                    other_car, f"OtherCar{i}_WaypointFollower",
                    VELOCITY_MAIN))
                # other_car_sequence.add_child(VehicleFollower(
                #     other_car,
                #     self._ego_car,
                #     EMirrors.DISTANCE_BETWEEN_CARS,
                #     name=f"OtherCar{i}_VehicleFollower"))
                trial.add_child(other_car_sequence)
            
            trials.add_child(trial)
        
        root.add_child(trials)

        distractor_sequence = Sequence("Distractor")
        distractor_sequence.add_child(DistractorSpawner(
            self._ego_car,
            DISTRACTOR,
            DISTRACTOR_LOCATIONS,
            distance=200,
            duration=7.5))

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


    ### ---------------------------------
    ### BEHAVIOURS
    ### ---------------------------------
    
    def _drive_until_close(self, vehicle, name, other, distance, velocity):
        """
        Wait until two cars approach each other
        """
        parallel = Parallel(f"{name}_FollowLaneUntilVehiclesAreClose",
                            policy=ParallelPolicy.SUCCESS_ON_ONE)
        parallel.add_children([
            WaypointFollower(
                vehicle,
                velocity,
                name=f"{name}_FollowLane"),
            InTriggerDistanceToVehicle(
                other,
                vehicle,
                distance=distance,
                comparison_operator=operator.lt,
                name=f"{name}_UntilVehiclesAreClose")
        ])

        return parallel

    def _drive_until_far(self, vehicle, name, other, distance, velocity):
        """
        Wait until two cars are apart of each other
        """
        parallel = Parallel(f"{name}_FollowLaneUntilVehiclesAreFar",
                            policy=ParallelPolicy.SUCCESS_ON_ONE)
        parallel.add_children([
            WaypointFollower(
                vehicle,
                velocity,
                name=f"{name}_FollowLane"),
            InTriggerDistanceToVehicle(
                other,
                vehicle,
                distance=distance,
                comparison_operator=operator.gt,
                name=f"{name}_UntilVehiclesAreApart")
        ])

        return parallel

    def _follow_waypoints(self, vehicle, name, velocity, duration=float("inf")):
        """
        Follows waypoints
        """
        parallel = Parallel(f"{name}_DrivingUtilTimeout",
                            policy=ParallelPolicy.SUCCESS_ON_ONE)

        # -- Follow the lane...
        parallel.add_child(WaypointFollower(
            vehicle,
            velocity,
            avoid_collision=True,
            name=f"{name}_LaneFollower"))

        # -- until timout or as long as the session continues
        parallel.add_child(Idle(duration, name=f"{name}_UntilTimeout"))

        return parallel

