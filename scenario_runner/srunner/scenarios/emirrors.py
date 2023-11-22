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


from py_trees.composites import (Parallel, Sequence)
from py_trees.common import ParallelPolicy

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      Idle,
                                                                      ChangeAutoPilot)
from srunner.scenariomanager.scenarioatomics.custom_behaviors import (DebugPrint,
                                                                      VehicleFollower)
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

class EMirrors(BasicScenario):

    """
    This class holds everything required for a "vehicle-passes-by" scenario involving two vehicles.
    There are two vehicles driving in the same direction on the highway: the ego-car in front, and 
    another (Opponent) car behind. The Opponen will accelerate after some time and pass by the ego car.
    """

    OPPONENT_SIDE_LEFT = 0
    OPPONENT_SIDE_RIGHT = 1

    DISTANCE_BETWEEN_CARS = 50              # meters
    DISTANCE_TO_APPROACH = 10               # meters
    DISTANCE_TO_OPPONENT_WHEN_FINISHED = 70 # meters

    DURATION_DRIVING_BEHIND = 50            # seconds

    VELOCITY_MAIN = 25.0                    # m/s
    VELOCITY_PASSING_BY = 5                 # m/s
    
    MAX_SPAWN_DISTANCE_FROM_REFERENCE = 5   # meters

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, params=""):
        """
        Setup all relevant parameters and create scenario
        """

        print(f"E-MIRRORS: params = {params}")

        self.timeout = timeout

        self._world = world
        self._map = CarlaDataProvider.get_map()

        # self._opponent_side = int(params) - 1
        
        self._ego_car = None                # carla.Vehicle
        # self._opponent = None               # carla.Vehicle
        # self._opponent_transform = None     # carla.Transform
        self._other_cars = []               # [(carla.Vehicle, carla.Transform)]

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
            
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            vehicle.set_simulate_physics(enabled=False)
            self.other_actors.append(vehicle)

            self._other_cars.append((vehicle, actor.transform))

    # override
    def _create_behavior(self):

        root = Parallel("Root", policy=ParallelPolicy.SUCCESS_ON_ONE)

        # Ego car
        ego_car_sequence = Sequence("Ego")
        ego_car_sequence.add_child(ChangeAutoPilot(
            self._ego_car,
            activate=True,
            name="Ego_AutoPilot",
            parameters={
                "auto_lane_change": False
            }))
        ego_car_sequence.add_child(self._follow_waypoints(
            self._ego_car, "Ego",
            EMirrors.VELOCITY_MAIN))
            
        # Opponent
        # opponent_sequence = Sequence("Opponent")
        # opponent_sequence.add_child(ActorTransformSetter(
        #     self._opponent,
        #     self._opponent_transform,
        #     name="Opponent_Placement"))
        # opponent_sequence.add_child(ChangeAutoPilot(
        #     self._opponent,
        #     activate=True,
        #     name="Opponent_AutoPilot",
        #     parameters={
        #         "auto_lane_change": False
        #     }))
        # opponent_sequence.add_child(DebugPrint(self._opponent, "== STAGE 1 - driving behind =="))

        # opponent_sequence.add_child(VehicleFollower(
        #     self._opponent,
        #     self._ego_car,
        #     EMirrors.DISTANCE_BETWEEN_CARS,
        #     EMirrors.DURATION_DRIVING_BEHIND,
        #     name=f"Opponent_VehicleFollower"))
        # opponent_sequence.add_child(DebugPrint(self._opponent, "== STAGE 2 - approaching =="))
        # opponent_sequence.add_child(self._drive_until_close(
        #     self._opponent, "Opponent",
        #     self._ego_car,
        #     EMirrors.DISTANCE_TO_APPROACH,
        #     EMirrors.VELOCITY_MAIN + EMirrors.VELOCITY_PASSING_BY))
        # opponent_sequence.add_child(DebugPrint(self._opponent, "== STAGE 3 - passing by =="))
        # opponent_sequence.add_child(self._drive_until_far(
        #     self._opponent, "Opponent",
        #     self._ego_car,
        #     EMirrors.DISTANCE_TO_OPPONENT_WHEN_FINISHED,
        #     EMirrors.VELOCITY_MAIN + EMirrors.VELOCITY_PASSING_BY))
        # opponent_sequence.add_child(DebugPrint(self._opponent, "== STAGE 2 - done =="))

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
                EMirrors.VELOCITY_MAIN))
            # other_car_sequence.add_child(VehicleFollower(
            #     other_car,
            #     self._ego_car,
            #     EMirrors.DISTANCE_BETWEEN_CARS,
            #     name=f"OtherCar{i}_VehicleFollower"))
            root.add_child(other_car_sequence)

        distractor_sequence = Sequence("Distractor")
        distractor_sequence.add_child(DistractorSpawner(
            self._ego_car,
            DISTRACTOR,
            DISTRACTOR_LOCATIONS,
            distance=200,
            duration=7.5))

        # Populate the root sequence
        root.add_child(ego_car_sequence)
        # root.add_child(opponent_sequence)
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

