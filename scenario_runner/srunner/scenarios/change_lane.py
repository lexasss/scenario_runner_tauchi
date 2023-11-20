#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
# Copyright (c) 2023 Tampere University
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Change lane scenario:

The scenario realizes a driving behavior, in which the user-controlled ego vehicle follows a car on the highway.
There're many other cars on the road but only two are able to change their lane (these cars are called as 'Opponents').
At one point the opponent car changes its lane; one the opponents does it at a safe distance, and the other provokes a collision.
"""

import carla

from py_trees.composites import (Parallel, Sequence)
from py_trees.common import ParallelPolicy

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      LaneChange,
                                                                      WaypointFollower,
                                                                      Idle,
                                                                      ChangeAutoPilot)
from srunner.scenariomanager.scenarioatomics.custom_behaviors import (DebugPrint,
                                                                      VehicleLightsControls,
                                                                      WaitForEvent,
                                                                      VehicleFollower)
# from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

from srunner.scenariomanager.timer import TimeOut

'''
SCENARIO
'''
class ChangeLane(BasicScenario):

    """
    This class holds everything required for a "change lane" scenario involving three vehicles.
    There are two vehicles driving in the same direction on the highway: A fast car and a slow car in front.
    The fast car will change the lane, when it is close to the slow car.

    The ego vehicle is driving right behind the fast Tesla car.

    This is a single ego vehicle scenario
    """

    LANE_CHANGE_ORDER_SAFE_UNSAFE = 1
    LANE_CHANGE_ORDER_UNSAFE_SAFE = 2

    TASK_JUST_DRIVING = 0
    TASK_LANE_CHANGE_SAFE = 1
    TASK_LANE_CHANGE_UNSAFE = 2

    TASK_NAME = ["just driving", "Safe lane change", "Unsafe lane change"]

    EVENT_SESSION_2_ENDS = 1
    EVENT_SESSION_3_ENDS = 2

    DRIVING_DURATION_SESSION = 342       # seconds
    DRIVING_DURATION_BEFORE_EVENT = 342  # seconds
    DRIVING_DURATION_AFTER_EVENT = 5     # seconds

    NUMBER_OF_OPPONENTS = 2;

    MAIN_VELOCITY = 25.0                    # m/s
    EXTRA_LANE_CHANGING_VELOCITY = 2              # m/s
    
    DISTANCE_BETWEEN_CARS_SAFE = 30         # meters
    DISTANCE_BETWEEN_CARS_UNSAFE = 8        # meters
    EXTRA_DISTANCE_BETWEEN_CARS_IN_DRIVING_BEHIND = 3    # meters

    # EGO_CAR_APPROACH_VELOCITY = 3           # m/s
    # MAX_SPAWN_DISTANCE_FROM_REFERENCE = 5   # meters

    ACCELERATION_SUPPRESSIONS = [       # list of speed-delta (m/s) and duration (seconds)
        (14, 15),
        (5, 15)
    ]

    # Lane change distance coefficient applied to Tesla's velocity:
    #   5.8 for normal
    #   3 for imminent collision
    LANE_CHANGE_DISTANCE_K = 4              # Oleg: weird behaiour for various K
    LANE_CHANGE_LIGHT_BLINK_DURATION = 2    # seconds
    
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, params=""):
        """
        Setup all relevant parameters and create scenario
        """

        self.timeout = timeout

        self._laneChangeOrder = int(params)
        self._map = CarlaDataProvider.get_map()
        
        self._ego_car = None        # carla.Vehicle
        self._opponents = []        # [(carla.Vehicle, carla.Transform)]
        self._other_cars = []       # [(carla.Vehicle, carla.Transform)]

        self._init_parameters()

        super(ChangeLane, self).__init__("ChangeLane",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)

        print(f"CHANGE LANE: params = {params}")
        
    # override
    def _initialize_actors(self, config):

        self._ego_car = self.ego_vehicles[0]

        # add actors from JSON file
        for actor in config.other_actors:
            vehicle_transform = actor.transform
            vehicle = CarlaDataProvider.request_new_actor(actor.model, vehicle_transform)
            vehicle.set_simulate_physics(enabled=False)
            self.other_actors.append(vehicle)

            # opponents are the first N cars in the list
            if len(self._opponents) < ChangeLane.NUMBER_OF_OPPONENTS:
                self._opponents.append((vehicle, vehicle_transform))
            
            # the rest cars will have no task other than simply following their lane
            if len(self.other_actors) > ChangeLane.NUMBER_OF_OPPONENTS:
                # TODO figure out why the next lines are needed
                # wp = self._map.get_waypoint(vehicle_transform.location)
                # wp, _ = get_waypoint_in_distance(wp, ChangeLane.MAX_SPAWN_DISTANCE_FROM_REFERENCE)
                # vehicle_transform = wp.transform

                self._other_cars.append((vehicle, vehicle_transform))

        print(f"CHANGE LANE: {len(self.other_actors)} surrounding cars initialized")

    # override
    def _create_behavior(self):

        root = Parallel("Root", policy=ParallelPolicy.SUCCESS_ON_ONE)

        ego_car_sequence = Sequence("Ego")
        opponent_sequences = []

        opponent_index = 1
        for (opponent, opponent_transform) in self._opponents:
            name = f"Opponent{opponent_index}"
            opponent_sequence = Sequence(name)
            opponent_sequence.add_child(self._init_vehicle(
                opponent, name,
                opponent_transform))
            opponent_sequences.append(opponent_sequence)
            opponent_index += 1

        # -- Just-driving task
        task = ChangeLane.TASK_JUST_DRIVING
        self._set_parameters(task=task)

        ego_car_sequence.add_child(DebugPrint(self._ego_car,
            "== SESSION 1 - JUST DRIVE =="))
        ego_car_sequence.add_child(self._session_just_drive_EGO_CAR())

        opponent_index = 1
        for opponent_sequence in opponent_sequences:
            name = f"Opponent{opponent_index}"
            opponent_sequence.add_child(self._session_just_drive_OPPONENT(
                self._opponents[opponent_index - 1][0],
                name))
            opponent_index += 1 

        # -- First lane-change task
        task = ChangeLane.TASK_LANE_CHANGE_SAFE \
                    if self._laneChangeOrder == ChangeLane.LANE_CHANGE_ORDER_SAFE_UNSAFE else \
                    ChangeLane.TASK_LANE_CHANGE_UNSAFE
        self._set_parameters(task=task)

        ego_car_sequence.add_child(DebugPrint(self._ego_car,
            f"== SESSION 2 - {ChangeLane.TASK_NAME[task]} =="))
        ego_car_sequence.add_child(self._session_change_lane_EGO_CAR(
            self._opponents[0][0],
            until_event=ChangeLane.EVENT_SESSION_2_ENDS))
        opponent_sequences[0].add_child(self._session_change_lane_OPPONENT(
            self._opponents[0][0], "Opponent1",
            is_safe=task == ChangeLane.TASK_LANE_CHANGE_SAFE,
            until_event=ChangeLane.EVENT_SESSION_2_ENDS))
        opponent_sequences[1].add_child(self._drive_straight_TESLA(
            self._opponents[1][0], "Opponent2",
            ChangeLane.MAIN_VELOCITY,
            until_event=ChangeLane.EVENT_SESSION_2_ENDS))

        # -- Second lane-change task
        task = ChangeLane.TASK_LANE_CHANGE_UNSAFE \
                    if self._laneChangeOrder == ChangeLane.LANE_CHANGE_ORDER_SAFE_UNSAFE else \
                    ChangeLane.TASK_LANE_CHANGE_SAFE
        self._set_parameters(task=task)

        ego_car_sequence.add_child(DebugPrint(self._ego_car,
            f"== SESSION 3 - {ChangeLane.TASK_NAME[task]} =="))
        ego_car_sequence.add_child(self._session_change_lane_EGO_CAR(
            self._opponents[1][0],
            until_event=ChangeLane.EVENT_SESSION_3_ENDS))
        opponent_sequences[0].add_child(self._drive_straight_TESLA(
            self._opponents[0][0], "Opponent1",
            ChangeLane.MAIN_VELOCITY,
            until_event=ChangeLane.EVENT_SESSION_3_ENDS))
        opponent_sequences[1].add_child(self._session_change_lane_OPPONENT(
            self._opponents[1][0], "Opponent2",
            is_safe=task == ChangeLane.TASK_LANE_CHANGE_SAFE,
            until_event=ChangeLane.EVENT_SESSION_3_ENDS))

        # -- Populate the root sequence
        root.add_child(ego_car_sequence)
        for opponent_sequence in opponent_sequences:
            root.add_child(opponent_sequence)
        
        # -- Behaviours for other cars
        carID = 1
        for (car, transform) in self._other_cars:
            velocity = ChangeLane.MAIN_VELOCITY

            # cars in the left-most lane move a bit faster
            # velocity = ChangeLane.MAIN_VELOCITY + \
            #                 (ChangeLane.EXTRA_LANE_CHANGING_VELOCITY \
            #                     if transform.location.y > 18 else
            #                 0)
            car_name = f"OtherCar{carID}"
            car_sequence = Sequence(car_name)
            car_sequence.add_child(self._init_vehicle(car, car_name, transform))
            car_sequence.add_child(self._drive_straight_TESLA(car, car_name, velocity))
            root.add_child(car_sequence)

            carID += 1
        
        return root

    '''
    def _setup_scenario_end(self, config):
        """
        Overrides the default condition that requires to end the route before finishing the scenario
        """
        return None
    '''

    # override
    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        #collision_criterion = CollisionTest(self.ego_vehicles[0])
        #criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


    ### ---------------------------------
    ### PARAMETERS
    ### ---------------------------------

    def _init_parameters(self):

        self._distance_between_cars = 0


    def _set_parameters(self, task):

        self._distance_between_cars = \
                ChangeLane.DISTANCE_BETWEEN_CARS_UNSAFE \
                    if task == ChangeLane.TASK_LANE_CHANGE_UNSAFE else \
                ChangeLane.DISTANCE_BETWEEN_CARS_SAFE


    ### ---------------------------------
    ### BEHAVIOURS
    ### ---------------------------------
    
    
    """
    LEVEL 1 (SESSIONS)
    """

    def _session_just_drive_EGO_CAR(self):
        """
        Creates ego behaviour for the first (just-drive) session
        """
        ego_car_sequence = Sequence("Ego_SessionDriveStraight")
        ego_car_sequence.add_children([
            DebugPrint(self._ego_car, "Ego SESSION just drive START"),
            self._drive_straight_EGO_CAR(
                False,
                ChangeLane.MAIN_VELOCITY,
                ChangeLane.DRIVING_DURATION_SESSION),
            DebugPrint(self._ego_car, "Ego SESSION just drive END")
        ])

        return ego_car_sequence

    def _session_just_drive_OPPONENT(self, opponent, name):
        """
        Creates opponent behaviour for the first (just-drive) session
        """
        opponent_sequence = Sequence(f"{name}_SessionDriveStraight")
        opponent_sequence.add_children([
            DebugPrint(opponent, f"{name} SESSION just drive START"),
            self._drive_straight_TESLA(
                opponent, name,
                ChangeLane.MAIN_VELOCITY,
                ChangeLane.DRIVING_DURATION_SESSION),
            DebugPrint(opponent, f"{name} SESSION just drive END")
        ])

        return opponent_sequence

    def _session_change_lane_EGO_CAR(self, opponent, until_event):
        """
        Creates ego behaviour for the 2nd/3rd (Lane Change) session
        """
        ego_car_sequence = Sequence("Ego_SessionChangeLane")
        ego_car_sequence.add_children([
            DebugPrint(self._ego_car, "Ego SESSION change lane START"),
            self._drive_behind(
                opponent,
                self._distance_between_cars + ChangeLane.EXTRA_DISTANCE_BETWEEN_CARS_IN_DRIVING_BEHIND,
                ChangeLane.DRIVING_DURATION_BEFORE_EVENT),
            DebugPrint(self._ego_car, "Ego start approaching"),
            self._get_close(self._ego_car, "Ego", opponent),
            # self._wait_until_close(
            #     self._ego_car, "Ego",
            #     opponent,
            #     ChangeLane.MAIN_VELOCITY + ChangeLane.EGO_CAR_APPROACH_VELOCITY),
            DebugPrint(self._ego_car, "Ego close enough, waiting until opponent changes a lane"),
            self._follow_waypoints(
                self._ego_car, "Ego",
                ChangeLane.MAIN_VELOCITY,
                event_id=until_event,
                avoid_collision=False),
            DebugPrint(self._ego_car, "Ego SESSION change lane END")
        ])

        return ego_car_sequence

    def _session_change_lane_OPPONENT(self, opponent, name, is_safe, until_event):
        """
        Creates opponent behaviour for the 2nd/3rd (Lane Change) session
        """
        opponent_sequence = Sequence(f"{name}_SessionChangeLane")
        opponent_sequence.add_children([
            DebugPrint(opponent, f"{name} SESSION change lane START"),
            self._follow_waypoints(
                opponent, name,
                ChangeLane.MAIN_VELOCITY,
                ChangeLane.DRIVING_DURATION_BEFORE_EVENT),
            DebugPrint(opponent, f"{name} waiting until Ego approaches"),
            self._wait_until_close(
                opponent, name,
                self._ego_car,
                ChangeLane.MAIN_VELOCITY),
            DebugPrint(opponent, f"{name} change lanes"),
            self._change_lane(
                opponent, name,
                is_blinker_enabled=is_safe),
            DebugPrint(opponent, f"{name} drive a bit further"),
            self._follow_waypoints(
                opponent, name,
                ChangeLane.MAIN_VELOCITY,
                ChangeLane.DRIVING_DURATION_AFTER_EVENT,
                event_id=until_event),
            DebugPrint(opponent, f"{name} SESSION change lane END")
        ])

        return opponent_sequence
    
    """
    LEVEL 2 (SUB-BEHAVIOUR)
    """

    def _init_vehicle(self, vehicle, name, transform):
        sequence = Sequence(f"{name}_Init")
        
        sequence.add_children([
            ActorTransformSetter(
                vehicle,
                transform,
                name=f"{name}_Placement"),
            ChangeAutoPilot(
                vehicle,
                activate=True,
                name=f"{name}_AutoPilot",
                parameters={
                    "auto_lane_change": False,
                    "distance_between_vehicles": 20,
                    "ignore_vehicles_percentage": 0
                })
        ])

        # sequence.add_child(DebugPrint(vehicle, f"{name} initialized"))

        return sequence

    def _drive_behind(self, opponent, distance, duration):
        """
        Ego cars enters autopilot mode and follows the an opponent until timeout
        """
        sequence = Sequence("Ego_DriveBehind")

        sequence.add_children([
            DebugPrint(self._ego_car, f"Ego drive behind START"),
            ChangeAutoPilot(
                self._ego_car,
                activate=True,
                name="Ego_AutoPilot",
                parameters={
                    "auto_lane_change": False,
                    "distance_between_vehicles": 5,
                    "ignore_vehicles_percentage": 0
                }
            ),
            VehicleFollower(self._ego_car, opponent, duration, distance),
            DebugPrint(self._ego_car, f"Ego drive behind END")
        ])

        return sequence

    def _drive_straight_EGO_CAR(self, is_autopilot, velocity, duration):
        """
        Ego cars enters autopilot mode and follows the lane waypoints until timeout
        """
        sequence = Sequence("Ego_DriveStraight")
        sequence.add_children([
            DebugPrint(self._ego_car, f"Ego drive straight START"),
            ChangeAutoPilot(
                self._ego_car,
                activate=is_autopilot,
                name="Ego_AutoPilot",
                parameters={
                    "auto_lane_change": False,
                    "distance_between_vehicles": 5,
                    "ignore_vehicles_percentage": 0
                }),
            DebugPrint(self._ego_car, f"Ego auto-pilot: {is_autopilot}"),
            self._follow_waypoints(
                self._ego_car, "Ego",
                velocity,
                duration),
            DebugPrint(self._ego_car, f"Ego drive straight END")
        ])

        return sequence

    def _drive_straight_TESLA(self, vehicle, name, velocity,
                              duration=None,
                              apply_slow_acceleration=False,    # currently never used with 'True'.. may be removed
                              until_event=None):
        """
        A car follows the lane waypoints until timeout or until ego/opponen car finish
        """
        sequence = Sequence(f"{name}_DriveStraight")
        sequence.add_child(DebugPrint(vehicle, f"{name} drive straight START"))

        duration_left = duration

        if apply_slow_acceleration:

            # -- Artificially slow down the car's acceleration to allow egocar to accelerate in sync
            for d_velocity, d_duration in ChangeLane.ACCELERATION_SUPPRESSIONS:

                # acceleration is a part of 'duration', therefore
                # the final driving period must exclude acceleration period
                if duration is not None:
                    duration_left -= d_duration 

                sequence.add_child(self._follow_waypoints(
                    vehicle, name,
                    velocity - d_velocity,
                    d_duration))

        # -- Continue driving normally
        sequence.add_child(self._follow_waypoints(
            vehicle, name,
            velocity,
            duration_left,
            event_id=until_event))
        sequence.add_child(DebugPrint(vehicle, f"{name} drive straight END"))

        return sequence

    def _get_close(self, vehicle, name, opponent):
        """
        Gets close to another car
        """
        parallel = Parallel(f"{name}_GetCloseToOpponent",
                            policy=ParallelPolicy.SUCCESS_ON_ONE)
        parallel.add_children([
            VehicleFollower(vehicle, opponent,
                duration=float("inf"),
                distance_between=self._distance_between_cars - 1),  # -1 ensures the car get close enough
            InTriggerDistanceToVehicle(
                opponent,
                vehicle,
                distance=self._distance_between_cars,
                name=f"{name}_UntilVehiclesAreClose")
        ])

        return parallel

    def _wait_until_close(self, vehicle, name, approaching_car, velocity):
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
                approaching_car,
                vehicle,
                distance=self._distance_between_cars,
                name=f"{name}_UntilVehiclesAreClose")
        ])

        return parallel

    def _change_lane(self, opponent, name, is_blinker_enabled):
        """
        Opponent changes 2 lanes
        """
        sequence = Sequence(f"{name}_ChangingLanes")

        # -- If needed: activate left blinker shortly before actually changing the lane
        if is_blinker_enabled:
            light_left_on = carla.VehicleLightState(carla.VehicleLightState.LeftBlinker)
            sequence.add_child(VehicleLightsControls(
                opponent,
                light_status=light_left_on,
                name=f"{name}_LightsOn"))
            sequence.add_child(TimeOut(ChangeLane.LANE_CHANGE_LIGHT_BLINK_DURATION))

        # -- Change 2 lanes at once
        # Note how the lane changing distance is dependent on the speed
        sequence.add_child(LaneChange(
            opponent,
            speed=ChangeLane.MAIN_VELOCITY + ChangeLane.EXTRA_LANE_CHANGING_VELOCITY,
            direction="left",
            distance_other_lane=ChangeLane.MAIN_VELOCITY * 1.5,
            distance_lane_change=ChangeLane.MAIN_VELOCITY * ChangeLane.LANE_CHANGE_DISTANCE_K,
            lane_changes=2,
            avoid_collision=False,
            name=f"{name}_ChangingLane"))

        # -- If needed: turn off all lights
        if is_blinker_enabled:
            light_off = carla.VehicleLightState(carla.VehicleLightState.NONE)
            sequence.add_child(VehicleLightsControls(
                opponent,
                light_status=light_off,
                name=f"{name}_LightsOff"))
        
        return sequence

    """
    LEVEL 3 (SUB-SUB-BEHAVIOUR)
    """

    def _follow_waypoints(self, vehicle, name, velocity, duration=None, event_id=None, avoid_collision=True):
        """
        Follows waypoints
        1. duration = None:
            a. event_id == None:  runs indefinitely (until the scenario ends)
            b. event_id == int:   until the global event is set
        2. duration > 0: 
            a. event_id == None:  until timeout, then finishes
            b. event_id == int:   until timeout, then sets the global event
        """
        parallel = Parallel(f"{name}_DrivingUtilTimeout",
                            policy=ParallelPolicy.SUCCESS_ON_ONE)

        # -- Follow the lane...
        parallel.add_child(WaypointFollower(
            vehicle,
            velocity,
            avoid_collision=avoid_collision,
            name=f"{name}_LaneFollower"))

        if duration is not None:
            if event_id is not None:
                # -- Drive for a certain period and set the global event at the end
                parallel.add_child(WaitForEvent(event_id, duration, name=name))
            else:
                # -- Drive for a certain period
                parallel.add_child(Idle(duration, name=f"{name}_UntilTimeout"))
        else:
            if event_id is not None:
                # -- Until the global event is set
                parallel.add_child(WaitForEvent(event_id, name=name))
            else:
                # -- Until ego/opponent exist
                parallel.add_child(Idle(name=f"{name}_UntilFinished"))

        return parallel
