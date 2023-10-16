#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Change lane scenario:

The scenario realizes a driving behavior, in which the user-controlled ego vehicle
follows a fast driving car on the highway. There's a slow car driving in great distance to the fast vehicle.
At one point the fast vehicle is changing the lane to overtake a slow car, which is driving on the same lane.

The ego vehicle doesn't "see" the slow car before the lane change of the fast car, therefore it hast to react
fast to avoid an collision. There are two options to avoid an accident:
The ego vehicle adjusts its velocity or changes the lane as well.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      LaneChange,
                                                                      WaypointFollower,
                                                                      Idle, AtomicBehavior,
                                                                      ChangeAutoPilot,
                                                                      ChangeActorTargetSpeed)
# from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

from srunner.scenariomanager.timer import TimeOut

class VehicleLightsControls(AtomicBehavior):
    """
    A class to set lights status on a vehicle

    Important parameters:
    - name: Name of the atomic behavior
    - actor: the vehicle that we are changing the light_status for
    - light_status: the new light state to be set
    """

    def __init__(self, actor, light_status=carla.VehicleLightState(carla.VehicleLightState.NONE), name="VehicleLights"):
        """
        Default init. Has to be called via super from derived class
        """
        super(VehicleLightsControls, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.name = name
        self._actor = actor
        self.light_status = light_status

    def update(self):
        self._actor.set_light_state(self.light_status)
        new_status = py_trees.common.Status.SUCCESS
        return new_status
    
    def initialise(self):
        return

class DebugPrint(AtomicBehavior):
    """
    A class to print out a message

    Important parameters:
    - actor: the vehicle that we are changing the light_status for
    - name: Name of the atomic behavior
    """

    is_enabled = True
    
    def __init__(self, actor, message, name="DebugPrint"):
        super(DebugPrint, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.name = name
        self._actor = actor
        self._message = message

    def update(self):
        if DebugPrint.is_enabled:
            print(f'[DEBUG]: {self._actor.type_id} > {self._message}')
        return py_trees.common.Status.SUCCESS

class ChangeLane(BasicScenario):

    """
    This class holds everything required for a "change lane" scenario involving three vehicles.
    There are two vehicles driving in the same direction on the highway: A fast car and a slow car in front.
    The fast car will change the lane, when it is close to the slow car.

    The ego vehicle is driving right behind the fast Tesla car.

    This is a single ego vehicle scenario
    """

    DRIVING_DURATION_FREE = 60          # seconds
    DRIVING_DURATION_AFTER_TEST = 5     # seconds

    TESLA_VELOCITY = 25.0           # m/s
    TESLA_OFFSET_FROM_REFERENCE = 5                     # meters
    SAFE_DISTANCE_BETWEEN_CARS_ON_LANE_CHANGE = 26      # meters
    UNSAFE_DISTANCE_BETWEEN_CARS_ON_LANE_CHANGE = 8     # meters
    EGO_CAR_APPROACH_VELOCITY = 1.5     # m/s
    EXTRA_RIGHT_LANE_SPEED = 1.3        # m/s

    ACCELERATION_SUPPRESSIONS = [       # list of speed-delta (m/s) and duration (seconds)
        (14, 15),
        (5, 15)
    ]

    # Lane change distance coefficient applied to Tesla's velocity:
    #   5.8 for normal
    #   3 for imminent collision
    LANE_CHANGE_DISTANCE_K = 3      # Oleg: weird behaiour for various K
    
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, params=''):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self.timeout = timeout
        self._task = params

        self._map = CarlaDataProvider.get_map()
        
        self._other_teslas = []

        self._tesla_offset_from_reference = ChangeLane.TESLA_OFFSET_FROM_REFERENCE
        self._tesla_velocity = ChangeLane.TESLA_VELOCITY
        self._egocar_velocity = ChangeLane.TESLA_VELOCITY + ChangeLane.EGO_CAR_APPROACH_VELOCITY
        self._distance_between_cars_on_lane_change = ChangeLane.UNSAFE_DISTANCE_BETWEEN_CARS_ON_LANE_CHANGE

        # if randomize:
        #     self._tesla_offset_from_reference = 1 + random.random() * 4
        #     self._tesla_velocity = random.randint(20, 30)
        
        if self._task == '1':     # just driver
            self._egocar_velocity = ChangeLane.TESLA_VELOCITY
        elif self._task == '2':   # safe
            self._distance_between_cars_on_lane_change = ChangeLane.SAFE_DISTANCE_BETWEEN_CARS_ON_LANE_CHANGE
        elif self._task == '3':   # unsafe
            pass

        super(ChangeLane, self).__init__("ChangeLane",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)

        print(f"CHANGE LANE: params = {params}")
        

    def _initialize_actors(self, config):

        tesla = None

        # add actors from xml file
        for actor in config.other_actors:
            if tesla is None:
                tesla = actor
            
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)
            
            if len(self.other_actors) > 1:
                wp = self._map.get_waypoint(actor.transform.location)
                wp, _ = get_waypoint_in_distance(wp, self._tesla_offset_from_reference)
                self._other_teslas.append((vehicle, wp.transform))

        # get Tesla's starting position
        location = tesla.transform.location
        if self._task == '1':     # in 'just driving', set Tesla a bit closer
            location.x += 10
        wp = self._map.get_waypoint(location)
        wp, _ = get_waypoint_in_distance(wp, self._tesla_offset_from_reference)
        self._tesla_transform = wp.transform
        
        print(f'CHANGE LANE: {len(self.other_actors)} surrounding cars initialized')

    def _create_behavior(self):

        tesla = self.other_actors[0]

        if self._task == '1':     # just drive
            duration = ChangeLane.DRIVING_DURATION_FREE - ChangeLane.ACCELERATION_DURATION_1 - ChangeLane.ACCELERATION_DURATION_2
            ego_car_sequence = self._create_ego_car_behaviour_drive_straight()
            tesla_sequence = self._create_behaviour_drive_straight(
                tesla, "Tesla", self._tesla_transform, self._tesla_velocity, duration=duration)
        elif self._task == '2':   # safe
            ego_car_sequence = self._create_ego_car_behaviour_drive_straight()
            tesla_sequence = self._create_tesla_behaviour_change_lane(show_lights=True)
        elif self._task == '3':   # unsafe
            ego_car_sequence = self._create_ego_car_behaviour_approach_tesla()
            tesla_sequence = self._create_tesla_behaviour_change_lane(show_lights=False)
        else:
            return None
        
        root = py_trees.composites.Parallel("CHANGE LANE Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(ego_car_sequence)
        root.add_child(tesla_sequence)
        
        carID = 1
        for (other_car, transform) in self._other_teslas:
            # cars in the left-most lane move a bit faster
            velocity = self._tesla_velocity + (ChangeLane.EXTRA_RIGHT_LANE_SPEED if transform.location.y > 18 else 0)
            root.add_child(self._create_behaviour_drive_straight(other_car, f'OtherCar{carID}', transform, velocity))
            carID += 1
        
        return root

    '''def _setup_scenario_end(self, config):
        """
        Overrides the default condition that requires to end the route before finishing the scenario
        """
        return None
    '''
    
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
    ### BEHAVIOURS
    ### ---------------------------------
    
    def _create_ego_car_behaviour_approach_tesla(self):
        
        ego_car = self.ego_vehicles[0]
        tesla = self.other_actors[0]

        ego_car_sequence = py_trees.composites.Sequence("Ego")

        ego_car_sequence.add_child(DebugPrint(ego_car, "start"))

        # Enable auto-pilot
        # Note that autopilot may overtake the control with some delay
        ego_car_autopilot = ChangeAutoPilot(ego_car, activate=True, name="Ego_AutoPilot")
        ego_car_sequence.add_child(ego_car_autopilot)
        ego_car_sequence.add_child(DebugPrint(ego_car, "auto-pilot enabled"))

        # --- Start parallel behaviour ---

        drive_until_tesla_reached = py_trees.composites.Parallel("Ego_AutoPilotUntilReachingTesla", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        """
        Out DrEyeVR car is much heavier than a regular Tesla, and therefore
        it must get a much higher speed in the beginning, but then 
        back to the normal speed

        So, we start first with reaching the 
        """

        # Follow the middle lane ... 
        p1 = py_trees.composites.Sequence("p1")
        p1.add_child(DebugPrint(ego_car, "lane-follower enabled"))
        lane_follower = WaypointFollower(ego_car, self._egocar_velocity, name="Ego_LaneFollower")
        p1.add_child(lane_follower)
        drive_until_tesla_reached.add_child(p1)

        # ...until tesla is close enough
        p2 = py_trees.composites.Sequence("p2")
        distance_to_tesla = InTriggerDistanceToVehicle(tesla, ego_car, distance=self._distance_between_cars_on_lane_change - 1, name="Ego_ApproachTesla")
        p2.add_child(distance_to_tesla)
        drive_until_tesla_reached.add_child(p2)

        ego_car_sequence.add_child(drive_until_tesla_reached)
        ego_car_sequence.add_child(DebugPrint(ego_car, "Tesla reached"))

        # ...then disable autopilot
        ego_car_manual = ChangeAutoPilot(ego_car, activate=False, name="Ego_ManualDriving")
        ego_car_sequence.add_child(ego_car_manual)
        ego_car_sequence.add_child(DebugPrint(ego_car, "manual driving enabled"))

        # Contunue running as long as Tesla exists
        # ego_car_sequence.add_child(WaitForEvent(name="Ego_UntilTeslaLeaves"))
        ego_car_sequence.add_child(Idle(name="Ego_UntilTeslaLeaves"))
        
        return ego_car_sequence
    
    def _create_ego_car_behaviour_drive_straight(self):
        
        ego_car = self.ego_vehicles[0]

        sequence = py_trees.composites.Sequence("Ego")

        sequence.add_child(DebugPrint(ego_car, "start"))

        # Enable auto-pilot
        # Note that autopilot may overtake the control with some delay
        ego_car_autopilot = ChangeAutoPilot(ego_car, activate=True, name="Ego_AutoPilot", parameters={
            'auto_lane_change': False,
            'distance_between_vehicles': 20,
            'ignore_vehicles_percentage': 0})
        sequence.add_child(ego_car_autopilot)
        sequence.add_child(DebugPrint(ego_car, "auto-pilot enabled"))

        # --- Constant-speed phase ---

        driving_straight = py_trees.composites.Parallel("Ego_DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Follow the middle lane...
        lane_follower = WaypointFollower(ego_car, self._egocar_velocity, avoid_collision=True, name="Ego_LaneFollower")
        driving_straight.add_child(lane_follower)

        # ...and continue running as long as Tesla exists
        # driving_straight.add_child(WaitForEvent(name="Ego_UntilTeslaLeaves"))
        driving_straight.add_child(Idle(name="Ego_UntilTeslaLeaves"))

        sequence.add_child(driving_straight)

        # --- end ---

        sequence.add_child(DebugPrint(ego_car, "Egocar leaving"))

        return sequence
    
    def _create_tesla_behaviour_change_lane(self, show_lights):
        
        ego_car = self.ego_vehicles[0]
        tesla = self.other_actors[0]
        
        # Sequence of Tesla actions, behaviours, triggers
        
        sequence = py_trees.composites.Sequence("Tesla")
        
        # Show Tesla in the starting location
        tesla_visible = ActorTransformSetter(tesla, self._tesla_transform, name="Tesla_Placement")
        sequence.add_child(tesla_visible)

        # Artificially slow down the Teslas' acceleration to allow egocar to accelerate in sync
        for d_velocity, d_duration in ChangeLane.ACCELERATION_SUPPRESSIONS:
            sequence.add_child(self._follow_waypoints(tesla, "Tesla", self._tesla_velocity - d_velocity, d_duration))

        # --- Start parallel behaviour ---

        # Move Tesla on the same lane ...
        driving_straight = py_trees.composites.Parallel("Tesla_DrivingUntilEgoIsClose", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        
        lane_follower = WaypointFollower(tesla, self._tesla_velocity, name="Tesla_LaneFollowerInitial")
        driving_straight.add_child(lane_follower)

        # ... for a certain distance before switching the left blinker on
        # distance_driven_before_lane_change = DriveDistance(tesla, ChangeLane.DISTANCE_TO_DRIVE_BEFORE_LANE_CHANGE, name="TeslaDrivingDisance")
        # driving_straight.add_child(distance_driven_before_lane_change)

        # ... until the ego car appears close enough
        distance_to_ego = InTriggerDistanceToVehicle(ego_car, tesla, distance=self._distance_between_cars_on_lane_change, name="Tesla_UntilEgoReaches")
        driving_straight.add_child(distance_to_ego)

        sequence.add_child(driving_straight)

        # --- end --

        if show_lights:
            # Tesla switched left turn blinking on
            light_left_on = carla.VehicleLightState(carla.VehicleLightState.LeftBlinker)
            sequence.add_child(VehicleLightsControls(tesla, light_status = light_left_on, name="Tesla_LightsOn"))
        
        # Tesla blinks the left turn for few second before changing the lane
        sequence.add_child(TimeOut(1.5))

        # Tesla changes 2 lanes at once
        # Note how the lane changing distance is dependent on the Tesla's speed
        lane_change = LaneChange(tesla,
                                 speed = self._tesla_velocity + ChangeLane.EXTRA_RIGHT_LANE_SPEED,
                                 distance_other_lane = self._tesla_velocity * 1.5,
                                 distance_lane_change = self._tesla_velocity * ChangeLane.LANE_CHANGE_DISTANCE_K,
                                 lane_changes = 2,
                                 name="Tesla_LaneChange")
        sequence.add_child(lane_change)

        if show_lights:
            # Tesla switches all lights off
            light_off = carla.VehicleLightState(carla.VehicleLightState.NONE)
            sequence.add_child(VehicleLightsControls(tesla, light_status=light_off, name="Tesla_LightsOff"))

        # --- Start parallel behaviour ---

        # And continues driving on the current (new) lane
        driving_straight = py_trees.composites.Parallel("Tesla_DrivingUntilFinished", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        
        lane_follower = WaypointFollower(tesla, self._tesla_velocity + ChangeLane.EXTRA_RIGHT_LANE_SPEED, name="Tesla_LaneFollowerFinal")
        driving_straight.add_child(lane_follower)

        # The trial is over after N seconds
        # driving_straight.add_child(WaitForEvent(timeout=5, name=f"Tesla_UntilTimeout"))
        driving_straight.add_child(Idle(ChangeLane.DRIVING_DURATION_AFTER_TEST, name=f"Tesla_FinalTimeout"))

        sequence.add_child(driving_straight)

        # --- end ---
        
        sequence.add_child(DebugPrint(tesla, "Tesla leaving"))
        
        return sequence
    
    def _create_behaviour_drive_straight(self, vehicle, name, transform, velocity, duration=None):
        
        sequence = py_trees.composites.Sequence(name)
        
        # Show Tesla in the starting location
        visible = ActorTransformSetter(vehicle, transform, name=f"{name}Placement")
        sequence.add_child(visible)

        # Enable auto-pilot
        autopilot = ChangeAutoPilot(vehicle, activate=True, name=f"{name}Pilot", parameters={
            # 'auto_lane_change': True,
            # 'force_lane_change': True,
            'distance_between_vehicles': 20,
            'ignore_vehicles_percentage': 0})
        sequence.add_child(autopilot)

        # Artificially slow down the Teslas' acceleration to allow egocar to accelerate in sync
        for d_velocity, d_duration in ChangeLane.ACCELERATION_SUPPRESSIONS:
            sequence.add_child(self._follow_waypoints(vehicle, name, velocity - d_velocity, d_duration))

        # --- Start parallel behaviour ---

        driving_straight = py_trees.composites.Parallel(f"{name}_DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Follow the lane
        lane_follower = WaypointFollower(vehicle, velocity, avoid_collision=True, name=f"{name}_LaneDriver")
        driving_straight.add_child(lane_follower)

        # Continue running ...
        if duration is not None:
            # ...for a certain period
            driving_straight.add_child(TimeOut(duration, name=f"{name}_UntilTimeout"))
            # driving_straight.add_child(WaitForEvent(timeout=duration, name=f"{name}_UntilTimeout"))
        else:
            # ...as long as Tesla exists
            driving_straight.add_child(Idle(name=f"{name}_UntilTeslaLeaves"))
            # driving_straight.add_child(WaitForEvent(name=f"{name}_UntilTeslaLeaves"))
            
        sequence.add_child(driving_straight)

        # --- end ---

        sequence.add_child(DebugPrint(vehicle, f"{name} leaving"))
        
        return sequence
    
    def _follow_waypoints(self, vehicle, name, velocity, duration):
        driving_for_duration = py_trees.composites.Parallel(f"{name}_FollowWaypointForDuration", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        lane_follower = WaypointFollower(vehicle, velocity, avoid_collision=True, name=f"{name}_LaneDriver_at_{velocity}")
        driving_for_duration.add_child(lane_follower)
        driving_for_duration.add_child(TimeOut(duration, name=f"{name}_DrivingDurationAt_{velocity}"))
        return driving_for_duration
