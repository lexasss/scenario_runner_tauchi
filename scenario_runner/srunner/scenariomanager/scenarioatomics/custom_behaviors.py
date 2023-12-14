import io
import carla
import math

from datetime import datetime

from typing import List, Dict, Optional

from py_trees.common import (Status as BehaviourStatus)

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (calculate_distance,
                                                                      WaypointFollower,
                                                                      AtomicBehavior)
from srunner.scenariomanager.timer import (GameTime, TimeOut)

from net.tcp_client import TcpClient

def print_debug(message):
    current_time = datetime.now().strftime("%H:%M:%S")
    print(f"{current_time} [DEBUG]: {message}")

class VehicleLightsControls(AtomicBehavior):
    """
    A class to set lights status on a vehicle

    Important parameters:
    - name: Name of the atomic behavior
    - actor: the vehicle that we are changing the light_status for
    - light_status: the new light state to be set
    """

    def __init__(self, actor,
                 light_status=carla.VehicleLightState(carla.VehicleLightState.NONE),
                 name="VehicleLights"):
        """
        Default init. Has to be called via super from derived class
        """
        self._actor: carla.Vehicle

        super(VehicleLightsControls, self).__init__(name, actor)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
        self.light_status = light_status

    def update(self):
        self._actor.set_light_state(self.light_status)
        return BehaviourStatus.SUCCESS
    
    def initialise(self):
        return

class DebugPrint(AtomicBehavior):
    """
    A class to print out a message

    Important parameters:
    - actor: the vehicle whose ID will be printed out
    - message to print out
    - name: Name of the atomic behavior
    """

    is_enabled = True
    
    def __init__(self, actor, message, name="DebugPrint"):
        super(DebugPrint, self).__init__(name, actor)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._message = message

    def update(self):
        if DebugPrint.is_enabled:
            print_debug(self._message)
        return BehaviourStatus.SUCCESS

class Log(AtomicBehavior):
    """
    A class to print out a message to a log file

    Important parameters:
    - actor: the vehicle whose ID will be printed out
    - message to print out
    - name: Name of the atomic behavior
    """

    file: io.TextIOWrapper
    
    def __init__(self, source, type, *data):
        super(Log, self).__init__("Log")
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        d = '\t'.join(data)
        self._message = f"{source}\t{type}\t{d}\n"

        if Log.file is None:
            Log.file = open(f'log_{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}.txt', 'w')

    def update(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        Log.file.write(f"{current_time}\t{self._message}")
        return BehaviourStatus.SUCCESS
    
    @staticmethod
    def close():
        Log.file.close()

class WaitForEvent(TimeOut):
    """
    Waits (keeps returning BehaviourStatus.RUNNING) until a given event is set.
    Note that events are global, so setting an event triggers to finilize all instances
    that are monitoring this event.

    Important parameters:
    - actor: the vehicle that we are changing the light_status for
    - event: id to set or to monitor
    - duration: used to flag the event (NOTE! only one instance should set the duration)
    - name: Name of the atomic behavior
    """

    _events = set(())
    
    def __init__(self, event, duration=float("inf"), name="WaitForEvent"):
        super(WaitForEvent, self).__init__(duration, name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
        self._duration = duration
        self._event = event

    def initialise(self):
        if self._duration != float("inf"):
            print_debug(f"{self.name} Event '{self._event}' will be set in {self._duration} s.")
        else:
            print_debug(f"{self.name} Waiting for event '{self._event}'")
        return super(WaitForEvent, self).initialise()

    def update(self):
        """
        Runs until
        1. the timeout that sets the global event
        2. the global event is set (by the timeout happened in another instance)
        """
        status = super(WaitForEvent, self).update()

        if status == BehaviourStatus.SUCCESS:
            print_debug(f"{self.name} Sets event '{self._event}'")
            WaitForEvent._events. add(self._event)
        elif self._event in WaitForEvent._events:
            print_debug(f"{self.name} Event '{self._event}' is set")
            status = BehaviourStatus.SUCCESS
        
        return status
    
    @staticmethod
    def reset():
        """
        Static method to reset the list of events
        """
        WaitForEvent._events = set(())

def sigmoid_0(value, k):
    return (2 / (1 + math.exp(-k * value) ) - 1)
def sign(value):
    return -1 if value < 0 else 1

class VehicleFollower(WaypointFollower):
    """
    The car will accelerate until it is faster than another car, in order to catch up distance.
    Then it tries to keep the distance, until timeout

    Important parameters:
    - actor: CARLA actor to execute the behaviour
    - other_actor: Reference CARLA actor, actor you want to catch up to
    - duration: total duration for catching up and following the vehicle
    - distance_between: distance between the actors, positive to keep the actor behind, and 
                        negative to keep the actor in front of the other_actor
    - delta_velocity: max additional velocity to speed up
    """

    def __init__(self, actor: carla.Actor, other_actor: carla.Actor, distance_between: float,
                 duration=float("inf"), 
                 delta_velocity=10.0,
                 name="VehicleFollower"):
        """
        Setup parameters
        """
        self._actor: carla.Vehicle
        super(VehicleFollower, self).__init__(actor, name=name,
                                              avoid_collision=True,
                                              local_planner_options={
                                                  "max_throttle": 0.85,
                                                  "max_brake": 0.2})

        self._other_actor = other_actor
        self._duration = duration
        self._distance_between = distance_between
        self._delta_velocity = delta_velocity  # 1m/s=3.6km/h

        self._prev_location = None

    def initialise(self):
        self._start_time = GameTime.get_time()
        super(VehicleFollower, self).initialise()

    def update(self):

        actor_location = CarlaDataProvider.get_location(self._actor)
        other_location = CarlaDataProvider.get_location(self._other_actor)
        
        if actor_location is None or other_location is None:
            return

        if self._prev_location is not None:
            # The Actor is behind the Other when the dot product
            # of Actor's direction and Actor-to-Other vector is positive
            dx1 = actor_location.x - self._prev_location.x
            dy1 = actor_location.y - self._prev_location.y
            dx2 = other_location.x - actor_location.x
            dy2 = other_location.y - actor_location.y
            dot_product = dx1 * dx2 + dy1 * dy2

            # is_behind = dot_product > 0

            # Distance is positive if the Actor is behind the Other, and
            # it is negative if the Actor is in front of the Other
            distance = actor_location.distance(other_location) * sign(dot_product)

            # actor_speed = CarlaDataProvider.get_velocity(self._actor)
            other_speed = CarlaDataProvider.get_velocity(self._other_actor)

            distance_error = distance - self._distance_between

            # Use max available speed increase when accelerating,
            delta_velocity = self._delta_velocity
            # but much less speed decrease when slowing down (it happens quite easily itself)
            if distance_error < 0:
                delta_velocity *= 0.35

            extra_velocity = delta_velocity * sigmoid_0(0.1, distance_error)
            new_speed = other_speed + extra_velocity

            # print(f"dist = {distance:.1f}, err = {distance_error:.1f}, speed = {actor_speed * 3.6:.1f}, accel = {extra_velocity * 3.6:.1f}, target-speed={new_speed * 3.6:.1f}")

            local_planner = self._local_planner_dict[self._actor]
            local_planner.set_speed(new_speed * 3.6)

        self._prev_location = actor_location

        new_status = BehaviourStatus.RUNNING

        if GameTime.get_time() - self._start_time > self._duration:
            new_status = BehaviourStatus.SUCCESS
        else:
            new_status = super(VehicleFollower, self).update()

        return new_status

CROSS_X = 9
CROSS_Y = 17
CROSS_SIZE = 100

class ApproachFromBehind(AtomicBehavior):
    """
    A class to implement the EMirror stury behaviour

    Important parameters:
    - name: Name of the atomic behavior
    - ego_car: the ego vehicle
    - other_cars: all other cars
    - distance: the distance for a car to approach to the ego vehicle
    - pause: time interval in seconds to drive before selecting a car to approach
    """

    def __init__(self, ego_car: carla.Vehicle,
                 other_cars: List[carla.Vehicle],
                 distance: float,
                 pause: float,
                 name="ApproachFromBehind"):
        """
        Default init. Has to be called via super from derived class
        """
        self._actor: carla.Vehicle
        
        super(ApproachFromBehind, self).__init__(name, ego_car)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
        self._other_cars = other_cars
        self._distance = distance
        self._pause = pause
        
        self._pause_end = 0
        self._approaching_car: Optional[carla.Vehicle] = None

    def update(self):
        if GameTime.get_time() < self._pause_end:
            # wait till the pause is finished
            return BehaviourStatus.RUNNING

        # choose the action depending of whether we have an apporaching car already
        
        if self._approaching_car is None:
            # the pause has ended: lets get the closest car running behind
            
            (closest_car, closest_car_dist) = self._get_closest_car_behind()
            if closest_car is None:
                return BehaviourStatus.RUNNING

            # we may have a car that is closer than needed
            if closest_car_dist < self._distance:
                print(f'AFTER PAUSE: Some cars are too close ({closest_car_dist:.1f} m). Waiting...')
                return BehaviourStatus.RUNNING
            
            # here we found that car: it is behind the ego car and is located further than the target distance.
            
            print(f'AFTER PAUSE: set an appoaching car {closest_car_dist:.1f} meters behind')
            self._approaching_car = closest_car

        else:
            # we have an approaching car, so lets check how close it is
            
            car_loc = CarlaDataProvider.get_location(self._approaching_car)
            ego_loc = CarlaDataProvider.get_location(self._actor)
            
            dist = calculate_distance(car_loc, ego_loc)
            if dist < self._distance:
                # the approaching car is close enough, let finish this behaviour
                print('APPROACHING: the car is close enough')
                return BehaviourStatus.SUCCESS
            else:
                print(f'APPROACHING: {dist:.1f}')

            # still too far away... lets continue running
            # hidden_transform = carla.Transform(carla.Location(250, -200, 10))
            # for car in self._other_cars:
            #     if car.is_alive:
            #         self._current_transforms[car] = car.get_transform()
                    
            #         car.set_target_velocity(carla.Vector3D(0, 0, 0))
            #         car.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
            #         car.set_transform(hidden_transform)

        return BehaviourStatus.RUNNING
   
    def initialise(self):
        self._pause_end = GameTime.get_time() + self._pause
        return
    
    # Internal
    
    def _get_closest_car_behind(self):
        ego_transform = CarlaDataProvider.get_transform(self._actor)
        if ego_transform is None:
            print('AFTER PAUSE: No ego car')
            return (None, 0)
        
        ego_loc = ego_transform.location
        
        # avoid the locations close to the cross
        if math.sqrt((ego_loc.x - CROSS_X)**2 + (ego_loc.y - CROSS_Y)**2) < CROSS_SIZE:
            print('AFTER PAUSE: Passing the cross')
            return (None, 0)
        
        # find the closest car running behind
        ego_yaw = ego_transform.rotation.yaw * math.pi / 180
        print(f'AFTER PAUSE: ego car yaw {ego_yaw*180/math.pi:.1f}')

        closest_dist = float("inf")
        closest_car: Optional[carla.Vehicle] = None
        
        for car in self._other_cars:
            car_transform = CarlaDataProvider.get_transform(car)
            if car_transform is None:
                continue
            
            car_loc = car_transform.location
            dx = ego_loc.x - car_loc.x
            dy = ego_loc.y - car_loc.y
            angle = math.atan2(dy, dx)
            
            # if the car is running behind the egocar, then egocar's 'yaw' value differs from
            # the angle of car->egocar vector no more than 70 degrees,
            # and therefore cos of this difference is > 0.2

            angle_diff = angle - ego_yaw
            if math.cos(angle_diff) < 0.2:
                continue
            
            dist = calculate_distance(car_loc, ego_loc)
            if dist < closest_dist:
                closest_dist = dist
                closest_car = car
                print(f'AFTER PAUSE: considering car at {dx:.1f}, {dy:.1f}, angle {angle*180/math.pi:.1f}')

        if closest_car is None:
            print('AFTER PAUSE: hmm...')
        else:
            print(f'AFTER PAUSE: selected car at the distance {closest_dist:.1f}')
                  
        return (closest_car, closest_dist)
    
class StopVehicleImmediately(AtomicBehavior):
    """
    Sets the vehicle target velocity

    Important parameters:
    - actor: the vehicle
    """

    def __init__(self, actor, name="StopVehicleImmediately"):
        self._actor: carla.Vehicle
        super(StopVehicleImmediately, self).__init__(name, actor)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self):
        self._actor.set_target_velocity(carla.Vector3D())
        return BehaviourStatus.SUCCESS


class EMirrorsScoresClient(AtomicBehavior):

    """
    Sends commands and events to the EMirrorsScores app

    Important parameters:
    - name: Name of the atomic behavior
    """

    client: Optional[TcpClient] = None

    def __init__(self, data, name="EMirrorsScoresClient"):
        """
        Default init. Has to be called via super from derived class
        """
        super(EMirrorsScoresClient, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
        self._data = data
        
        if EMirrorsScoresClient.client is None:
            EMirrorsScoresClient.client = TcpClient('192.168.1.130')
            EMirrorsScoresClient.client.connect(lambda x: print(f'[TCP] {x}'))

    def update(self):
        if EMirrorsScoresClient.client:
            EMirrorsScoresClient.client.send(self._data)
            
            if self._data == 'stop':
                EMirrorsScoresClient.client.close()
            
        return BehaviourStatus.SUCCESS

    def terminate(self, new_status):
        """
        Connection termination
        """
        if EMirrorsScoresClient.client:
            EMirrorsScoresClient.client.close()
            EMirrorsScoresClient.client = None
            
        super(EMirrorsScoresClient, self).terminate(new_status)

