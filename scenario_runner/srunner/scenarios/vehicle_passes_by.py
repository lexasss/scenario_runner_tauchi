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
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

class VehiclePassesBy(BasicScenario):

    """
    This class holds everything required for a "vehicle-passes-by" scenario involving two vehicles.
    There are two vehicles driving in the same direction on the highway: the ego-car in front, and 
    another (Opponent) car behind. The Opponen will accelerate after some time and pass by the ego car.
    """

    OPPONENT_SIDE_LEFT = 0
    OPPONENT_SIDE_RIGHT = 1

    OPPONENT_SIDE_NAME = ["Left", "Right"]

    DISTANCE_BETWEEN_CARS = 50              # meters
    DISTANCE_TO_APPROACH = 10               # meters
    DISTANCE_TO_OPPONENT_WHEN_FINISHED = 40 # meters

    DRIVING_DURATION_BEHIND = 45            # seconds
    DRIVING_DURATION_PASSING_BY = 40        # seconds

    VELOCITY_MAIN = 25.0                    # m/s
    VELOCITY_PASSING_BY = 5                 # m/s
    
    MAX_SPAWN_DISTANCE_FROM_REFERENCE = 5   # meters

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, params=""):
        """
        Setup all relevant parameters and create scenario
        """

        self.timeout = timeout

        self._map = CarlaDataProvider.get_map()

        self._opponent_side = int(params) - 1
        
        self._ego_car = None                # carla.Vehicle
        self._opponent = None               # carla.Vehicle
        self._opponent_transform = None     # carla.Transform

        super(VehiclePassesBy, self).__init__("VehiclePassesBy",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

        print(f"VEHICLE PASSES BY: params = {params}")
        
    # override
    def _initialize_actors(self, config):

        self._ego_car = self.ego_vehicles[0]

        # add actors from JSON file
        for actor in config.other_actors:
            
            print(actor.model)
            if (self._opponent_side == VehiclePassesBy.OPPONENT_SIDE_LEFT and actor.transform.location.y > 12.7) or \
               (self._opponent_side == VehiclePassesBy.OPPONENT_SIDE_RIGHT and actor.transform.location.y < 12.7):
               
                vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
                vehicle.set_simulate_physics(enabled=False)
                self.other_actors.append(vehicle)

                self._opponent = vehicle

                # TODO figure out why the next line is needed
                # wp = self._map.get_waypoint(actor.transform.location)
                # wp, _ = get_waypoint_in_distance(wp, ChangeLane.MAX_SPAWN_DISTANCE_FROM_REFERENCE)
                # self._opponent_transform = wp.transform

                self._opponent_transform = actor.transform

                print(f"VEHICLE PASSES BY: opponent initialized")

                # Ignore any other vehicles
                break

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
            VehiclePassesBy.VELOCITY_MAIN))
            
        # Opponent
        opponent_sequence = Sequence("Opponent")
        opponent_sequence.add_child(ActorTransformSetter(
            self._opponent,
            self._opponent_transform,
            name="Opponent_Placement"))
        opponent_sequence.add_child(ChangeAutoPilot(
            self._opponent,
            activate=True,
            name="Opponent_AutoPilot",
            parameters={
                "auto_lane_change": False
            }))
        opponent_sequence.add_child(DebugPrint(self._opponent, "== STAGE 1 - driving behind =="))

        opponent_sequence.add_child(VehicleFollower(
            self._opponent,
            self._ego_car,
            VehiclePassesBy.DISTANCE_BETWEEN_CARS,
            VehiclePassesBy.DRIVING_DURATION_BEHIND))
        opponent_sequence.add_child(DebugPrint(self._opponent, "== STAGE 2 - approaching =="))
        opponent_sequence.add_child(self._drive_until_close(
            self._opponent, "Opponent",
            self._ego_car,
            VehiclePassesBy.DISTANCE_TO_APPROACH,
            VehiclePassesBy.VELOCITY_MAIN + VehiclePassesBy.VELOCITY_PASSING_BY))
        opponent_sequence.add_child(DebugPrint(self._opponent, "== STAGE 3 - passing by =="))
        opponent_sequence.add_child(self._drive_until_far(
            self._opponent, "Opponent",
            self._ego_car,
            VehiclePassesBy.DISTANCE_TO_OPPONENT_WHEN_FINISHED,
            VehiclePassesBy.VELOCITY_MAIN + VehiclePassesBy.VELOCITY_PASSING_BY))
        opponent_sequence.add_child(DebugPrint(self._opponent, "== STAGE 2 - done =="))

        # Populate the root sequence
        root.add_child(ego_car_sequence)
        root.add_child(opponent_sequence)
        
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
