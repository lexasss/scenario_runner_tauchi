# Behaviours

## AtomicBehavior

Base class for all atomic behaviors used to setup a scenario. Derived from py_trees.behaviour.Behaviour

*All behaviors should use this class as parent*

<details>
<summary>Template</summary>
Args:
- `name`: Name of the atomic behavior

``` python

    def __init__(self, name, actor=None):
        # Default init. Has to be called via super from derived class
        super(AtomicBehavior, self).__init__(name)
        self.name = name
        self._actor = actor

    def setup(self, unused_timeout=15):
        # Default setup
        return True

    def initialise(self):
        # Initialise setup terminates WaypointFollowers

    def terminate(self, new_status):
        # Default terminate. Can be extended in derived class
```
</details>

## RunScript

This is an atomic behavior to start execution of an additional script.

<details>
<summary>Details</summary>

Args:
- `script` (str): String containing the interpreter, scriptpath and arguments.
        Example: "python /path/to/script.py --arg1"
- `base_path` (str): String containing the base path of for the script

Attributes:
- `_script` (str): String containing the interpreter, scriptpath and arguments
        Example: "python /path/to/script.py --arg1"
- `_base_path` (str): String containing the base path of for the script
        Example: "/path/to/"

**Note**: This is intended for the use with OpenSCENARIO. Be aware of security side effects.
</details>

## ChangeParameter
    
This is an atomic behavior to change the OSC parameter value.

<details>
<summary>Details</summary>

Args:
- `parameter_ref` (str): parameter name
- `value` (any): ParameterRef or number

</details>

## ChangeWeather

Atomic to write a new weather configuration into the blackboard.
Used in combination with WeatherBehavior() to have a continuous weather simulation.

<details>
<summary>Details</summary>

The behavior immediately terminates with SUCCESS after updating the blackboard.

Args:
- `weather` (srunner.scenariomanager.weather_sim.Weather): New weather settings.

</details>

## ChangeRoadFriction

Atomic to update the road friction in CARLA.

<details>
<summary>Details</summary>

The behavior immediately terminates with SUCCESS after updating the friction.

Args:
- `friction` (float): New friction coefficient.

</details>

## ChangeActorControl

Atomic to change the longitudinal/lateral control logic for an actor.
The (actor, controller) pair is stored inside the Blackboard.

<details>
<summary>Details</summary>

The behavior immediately terminates with SUCCESS after the controller.

Args:
- `actor` (carla.Actor): Actor that should be controlled by the controller.
- `control_py_module` (str): Name of the python module containing the implementation of the controller.
- `args` (dictionary): Additional arguments for the controller.
- `scenario_file_path` (str): Additional path to controller implementation.

Attributes:
- `_actor_control` (ActorControl): Instance of the actor control.

</details>

## UpdateAllActorControls

Atomic to update (run one control loop step) all actor controls.

<details>
<summary>Details</summary>

The behavior is always in RUNNING state.

</details>

## ChangeActorTargetSpeed

Atomic to change the target speed for an actor controller.

<details>
<summary>Details</summary>

The behavior is in RUNNING state until the distance/duration
conditions are satisfied, or if a second `ChangeActorTargetSpeed` atomic
for the same actor is triggered.

Args:
- `actor` (carla.Actor): Controlled actor.
- `target_speed` (float): New target speed [m/s].
- `init_speed` (bool): Flag to indicate if the speed is the initial actor speed.
        Defaults to False.
- `duration` (float): Duration of the maneuver [s].
        Defaults to None.
- `distance` (float): Distance of the maneuver [m].
        Defaults to None.
- `relative_actor` (carla.Actor): If the target speed setting should be relative to another actor.
        Defaults to None.
- `value` (float): Offset, if the target speed setting should be relative to another actor.
        Defaults to None.
- `value_type` (str): Either 'Delta' or 'Factor' influencing how the offset to the reference actors velocity is applied. Defaults to None.
- `continuous` (bool): If True, the atomic remains in RUNNING, independent of duration or distance.
        Defaults to False.

Attributes:
- `_start_time` (float): Start time of the atomic [s].
        Defaults to None.
- `_start_location` (carla.Location): Start location of the atomic.
        Defaults to None.

</details>

## SyncArrivalOSC

Atomic to make two actors arrive at their corresponding places at the same time

<details>
<summary>Details</summary>

The behavior is in RUNNING state until the "main" actor has reached its destination

Args:
- `actor` (carla.Actor): Controlled actor.
- `master_actor` (carla.Actor): Reference actor to sync up to.
- `actor_target` (carla.Transform): Endpoint of the actor after the behavior finishes.
- `master_target` (carla.Transform): Endpoint of the `master_actor` after the behavior finishes.
- `final_speed` (float): Speed of the actor after the behavior ends.
- `relative_to_master` (bool): Whether or not the final speed is relative to `master_actor`.
        Defaults to False.
- `relative_type` (str): Type of relative speed. Either 'delta' or 'factor'.
        Defaults to ''.

</details>

## ChangeActorWaypoints

Atomic to change the waypoints for an actor controller.

<details>
<summary>Details</summary>

The behavior is in RUNNING state until the last waypoint is reached, or if a
second waypoint related atomic for the same actor is triggered. These are:
- `ChangeActorWaypoints`
- `ChangeActorLateralMotion`
- `ChangeActorLaneOffset`

Args:
- `actor` (carla.Actor): Controlled actor.
- `waypoints` (List of (OSC position, OSC route option)): List of (Position, Route Option) as OpenScenario elements. Position will be converted to Carla transforms, considering the corresponding route option (e.g. shortest, fastest)

Attributes:
- `_start_time` (float): Start time of the atomic [s].
        Defaults to None.

**Note**: When using routing options such as fastest or shortest, it is advisable to run in synchronous mode

</details>

## ChangeActorLateralMotion

Atomic to change the waypoints for an actor controller.

<details>
<summary>Details</summary>

The behavior is in RUNNING state until the last waypoint is reached, or if a
second waypoint related atomic for the same actor is triggered. These are:
- `ChangeActorWaypoints`
- `ChangeActorLateralMotion`
- `ChangeActorLaneOffset`

If an impossible lane change is asked for (due to the lack of lateral lanes,
next waypoints, continuous line, etc) the atomic will return a plan with the
waypoints until such impossibility is found.

Args:
- `actor` (carla.Actor): Controlled actor.
- `direction` (str): Lane change direction ('left' or 'right').
        Defaults to 'left'.
- `distance_lane_change` (float): Distance of the lance change [meters].
        Defaults to 25.
- `distance_other_lane` (float): Driven distance after the lange change [meters].
        Defaults to 100.

Attributes:
- `_waypoints` (List of carla.Transform): List of waypoints representing the lane change (CARLA transforms).
- `_distance_same_lane` (float): Distance on the same lane before the lane change starts [meters]
        Constant to 5.
- `_pos_before_lane_change`: carla.Location of the actor before the lane change.
        Defaults to None.
- `_target_lane_id` (int): Id of the target lane
        Defaults to None.
- `_start_time` (float): Start time of the atomic [s].
        Defaults to None.

</details>

## ChangeActorLaneOffset

*OpenSCENARIO atomic.*

Atomic to change the offset of the controller.

<details>
<summary>Details</summary>

The behavior is in RUNNING state until the offset os reached (if `continuous` is set to False) or forever (if `continuous` is True). This behavior will automatically stop if a second waypoint related atomic for the same actor is triggered. These are:
- `ChangeActorWaypoints`
- `ChangeActorLateralMotion`
- `ChangeActorLaneOffset`

Args:
- `actor` (carla.Actor): Controlled actor.
- `offset` (float): Float determined the distance to the center of the lane. Positive distance imply a displacement to the right, while negative displacements are to the left.
- `relative_actor` (carla.Actor): The actor from which the offset is taken from. Defaults to None
- `continuous` (bool): If True, the behaviour never ends. If False, the behaviour ends when the lane offset is reached. Defaults to True.

Attributes:
- `_start_time` (float): Start time of the atomic [s].
        Defaults to None.
- `_overwritten` (bool): flag to check whether or not this behavior was overwritten by another. Helps to avoid the missinteraction between two ChangeActorLaneOffsets.
- `_current_target_offset` (float): stores the value of the offset when dealing with relative distances
- `_map` (carla.Map): instance of the CARLA map.

</details>

## ActorTransformSetterToOSCPosition

This class contains an atomic behavior to set the transform of an OpenSCENARIO actor.

<details>
<summary>Details</summary>

The behavior terminates when actor is set to the new actor transform (closer than 1 meter)

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `osc_position`: OpenSCENARIO position
- `physics` (bool) [optional]: If physics is true, the actor physics will be reactivated upon success

**Note**: It is very important to ensure that the actor location is spawned to the new transform because of the appearence of a rare runtime processing error. `WaypointFollower` with `LocalPlanner`, might fail if new_status is set to success before the actor is really positioned at the new transform. Therefore: `calculate_distance(actor, transform) < 1 meter`

</details>

## AccelerateToVelocity

This class contains an atomic acceleration behavior. The controlled
traffic participant will accelerate with `_throttle_value_` until reaching
a given `_target_velocity_`

<details>
<summary>Details</summary>

The behavior will terminate, if the actor's velocity is at least target_velocity

Args:
- `actor`(carla.Actor): CARLA actor to execute the behavior
- `throttle_value` (float): The amount of throttle used to accelerate in [0,1]
- `target_velocity` (float): The target velocity the actor should reach in m/s

</details>

## AccelerateToCatchUp

This class contains an atomic acceleration behavior.
The car will accelerate until it is faster than another car, in order to catch up distance. This behaviour is especially useful before a lane change (e.g. LaneChange atom).

<details>
<summary>Details</summary>

The behaviour will terminate succesful, when the two actors are in `trigger_distance`. If max_distance is driven by the actor before actors are in `trigger_distance`, then the behaviour ends with a failure.

Args:
- `actor` (carla.Actor): CARLA actor to execute the behaviour
- `other_actor`(carla.Actor): Reference CARLA actor, actor you want to catch up to
- `throttle_value` (float): acceleration value between 0.0 and 1.0
- `delta_velocity` (float): speed up to the velocity of other actor plus delta_velocity
- `trigger_distance` (float): distance between the actors
- `max_distance` (float): driven distance to catch up has to be smaller than max_distance

</details>

## KeepVelocity

This class contains an atomic behavior to keep the provided velocity.
The controlled traffic participant will accelerate as fast as possible
until reaching a given _target_velocity_, which is then maintained for
as long as this behavior is active.

<details>
<summary>Details</summary>

A termination can be enforced by providing distance or duration values.
Alternatively, a parallel termination behavior has to be used.

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `target_velocity` (float): The target velocity the actor should reach
- `duration` (float) [optional]: Duration in seconds of this behavior
- `distance` (float) [optional]: Maximum distance in meters covered by the actor during this behavior

</details>

## ChangeAutoPilot

This class contains an atomic behavior to disable/enable the use of the autopilot.

<details>
<summary>Details</summary>

The behavior terminates after changing the autopilot state

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `activate` (bool): True (=enable autopilot) or False (=disable autopilot)
- `lane_change` (bool): Traffic Manager parameter. True (=enable lane changes) or False (=disable lane changes)
- `distance_between_vehicles` (float): Traffic Manager parameter
- `max_speed` (float): Traffic Manager parameter. Max speed of the actor. This will only work for road segments with the same speed limit as the first one

</details>

## StopVehicle

This class contains an atomic stopping behavior. The controlled traffic
participant will decelerate with _bake_value_ until reaching a full stop.

<details>
<summary>Details</summary>

The behavior terminates when the actor stopped moving

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `brake_value` (float): Brake value in [0,1] applied

</details>

## SyncArrival

This class contains an atomic behavior to
set velocity of actor so that it reaches location at the same time as
actor_reference. The behavior assumes that the two actors are moving
towards location in a straight line.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `actor_reference` (carla.Actor): Reference actor with which arrival is synchronized
- `target_location` (carla.Location): CARLA location where the actors should "meet"
- `gain` (float) [optional]: Coefficient for actor's throttle and break controls

**Note**: In parallel to this behavior a termination behavior has to be used to keep continue synchronization for a certain duration, or for a certain distance, etc.

</details>

## AddNoiseToVehicle

This class contains an atomic jitter behavior.
To add noise to steer as well as throttle of the vehicle.

<details>
<summary>Details</summary>

The behavior terminates after setting the new actor controls

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `steer_value` (float): Applied steering noise in [0,1]
- `throttle_value` (float): Applied throttle noise in [0,1]

</details>

## ChangeNoiseParameters

This class contains an atomic jitter behavior.
To add noise to steer as well as throttle of the vehicle.

<details>
<summary>Details</summary>

This behavior should be used in conjuction with `AddNoiseToVehicle`.

The behavior terminates after one iteration

</details>

## BasicAgentBehavior

This class contains an atomic behavior, which uses the `basic_agent` from CARLA to control the actor until reaching a target location.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `target_location` (carla.Location): Is the desired target location, the actor should move to

The behavior terminates after reaching the `target_location` (within 2 meters)

</details>

## Idle

This class contains an idle behavior scenario

<details>
<summary>Details</summary>

A termination can be enforced by providing a duration value.
Alternatively, a parallel termination behavior has to be used.

Args:
- `duration` (float) [optional]: Duration in seconds of this behavior

</details>

## WaypointFollower

This is an atomic behavior to follow waypoints while maintaining a given speed.
If no plan is provided, the actor will follow its foward waypoints indefinetely. Otherwise, the behavior will end with SUCCESS upon reaching the end of the plan. If no target velocity is provided, the actor continues with its current velocity.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor):  CARLA actor to execute the behavior.
- `target_speed` (float) [optional]: Desired speed of the actor in m/s. Defaults to None.
- `plan` ([carla.Location] or [(carla.Waypoint, carla.agent.navigation.local_planner)]) [optional]:
        Waypoint plan the actor should follow. Defaults to None.
- `blackboard_queue_name` (str) [optional]:
        Blackboard variable name, if additional actors should be created on-the-fly. Defaults to None.
- `avoid_collision` (bool) [optional]:
        Enable/Disable(=default) collision avoidance for vehicles/bikes. Defaults to False.

Attributes:
- `_actor_dict`: Dictonary of all actors, and their corresponding plans (e.g. {actor: plan}).
- `_local_planner_dict`: Dictonary of all actors, and their corresponding local planners.
        Either "Walker" for pedestrians, or a carla.agent.navigation.LocalPlanner for other actors.
- `_args_lateral_dict`: Parameters for the PID of the used carla.agent.navigation.LocalPlanner.
- `_unique_id`: Unique ID of the behavior based on timestamp in nanoseconds.

**Note**: OpenScenario: The `WaypointFollower` atomic must be called with an individual name if multiple consecutive WFs. Blackboard variables with lists are used for consecutive `WaypointFollower` behaviors. Termination of active `WaypointFollowers` in initialise of `AtomicBehavior` because any following behavior must terminate the `WaypointFollower`.

</details>

## LaneChange

This class inherits from the class `WaypointFollower`.

This class contains an atomic lane change behavior to a parallel lane.
The vehicle follows a waypoint plan to the other lane, which is calculated in the initialise method. This waypoint plan is calculated with a scenario helper function.

If an impossible lane change is asked for (due to the lack of lateral lanes,
next waypoints, continuous line, etc) the atomic will return a plan with the
waypoints until such impossibility is found.

<details>
<summary>Details</summary>


The total distance driven is greater than the sum of `distance_same_lane` and `distance_other_lane`. It results from the lane change distance plus the distance_same_lane plus `distance_other_lane`.
The lane change distance is set to 25m (straight), the driven distance is slightly greater.

A parallel termination behavior has to be used.

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `speed` (float): speed of the actor for the lane change, in m/s
- `direction` (str): 'right' or 'left', depending on which lane to change
- `distance_same_lane` (float): straight distance before lane change, in m
- `distance_other_lane` (float): straight distance after lane change, in m
- `distance_lane_change` (float): straight distance for the lane change itself, in m
</details>

## SetInitSpeed

This class contains an atomic behavior to set the init_speed of an actor,
succeding immeditely after initializing

## HandBrakeVehicle

This class contains an atomic hand brake behavior.
To set the hand brake value of the vehicle.

<details>
<summary>Details</summary>

The behavior terminates after setting the hand brake value

Args:
- `vehicle` (carla.Actor): CARLA actor to execute the behavior
- `hand_brake_value` (float): to be applied in [0,1]

</details>

## ActorDestroy

This class contains an actor destroy behavior.
Given an actor this behavior will delete it.

<details>
<summary>Details</summary>

The behavior terminates after removing the actor

Args:
- `actor` (carla.Actor): CARLA actor to be deleted

</details>

## ActorTransformSetter

This class contains an atomic behavior to set the transform
of an actor.

<details>
<summary>Details</summary>

The behavior terminates when actor is set to the new actor transform (closer than 1 meter)

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- transform (carla.Transform): New target transform (position + orientation) of the actor
- `physics` (bool) [optional]: If physics is true, the actor physics will be reactivated upon success

**Note**: It is very important to ensure that the actor location is spawned to the new transform because of the appearence of a rare runtime processing error. WaypointFollower with `LocalPlanner`, might fail if new_status is set to success before the actor is really positioned at the new transform. Therefore: `calculate_distance(actor, transform) < 1 meter`

</details>

## TrafficLightStateSetter

This class contains an atomic behavior to set the state of a given traffic light

<details>
<summary>Details</summary>

The behavior terminates after trying to set the new state

Args:
- `actor` (carla.TrafficLight): ID of the traffic light that shall be changed
- `state` (carla.TrafficLightState): New target state

</details>

## ActorSource

Implementation for a behavior that will indefinitely create actors
at a given transform if no other actor exists in a given radius
from the transform.

<details>
<summary>Details</summary>

A parallel termination behavior has to be used.

Args:
- `actor_type_list` (str[]): Type of CARLA actors to be spawned
- `transform` (carla.Transform): Spawn location
- `threshold` (float): Min available free distance between other actors and the spawn location
- `blackboard_queue_name` (str): Name of the blackboard used to control this behavior
- `actor_limit` (float) [optional]: Maximum number of actors to be spawned (default=7)

</details>

## ActorSink

Implementation for a behavior that will indefinitely destroy actors
that wander near a given location within a specified threshold.

<details>
<summary>Details</summary>

A parallel termination behavior has to be used.

Args:
- `actor_type_list` (list(str)): Type of CARLA actors to be spawned
- `sink_location` (carla.Location): Location at which actors will be deleted
- `threshold` (float): Distance around sink_location in which actors will be deleted

</details>

## StartRecorder

Atomic that starts the CARLA recorder. Only one can be active
at a time, and if this isn't the case, the recorder will
automatically stop the previous one.

<details>
<summary>Details</summary>

Args:
- `recorder_name` (str): name of the file to write the recorded data.
        Remember that a simple name will save the recording in
        'CarlaUE4/Saved/'. Otherwise, if some folder appears in the name,
        it will be considered an absolute path.

</details>

## StopRecorder

Atomic that stops the CARLA recorder.

## TrafficLightManipulator

Atomic behavior that manipulates traffic lights around the `ego_vehicle` to trigger scenarios 7 to 10. This is done by setting 2 of the traffic light at the intersection to green (with some complex precomputation to set everything up).

<details>
<summary>Details</summary>

Args:
- `ego_vehicle` (carla.Actor): CARLA actor that controls this behavior
- `subtype` (str): string that gathers information of the route and scenario number (check SUBTYPE_CONFIG_TRANSLATION below)

</details>

## ScenarioTriggerer

Handles the triggering of the scenarios that are part of a route.

<details>
<summary>Details</summary>

Initializes a list of blackboard variables to False, and only sets them to True when the ego vehicle is very close to the scenarios

</details>

## KeepLongitudinalGap

This class contains an atomic behavior to maintain a set gap with leading/adjacent vehicle.

<details>
<summary>Details</summary>

The behavior terminates after overwritten by other events / when target distance is reached(if continues).

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `reference_actor` (carla.Actor): Reference actor the distance shall be kept to.
- `distance` (float): target gap between the two actors in meters
- `distance_type` (str): Specifies how distance should be calculated between the two actors

</details>

---

# Triggers

## AtomicCondition

Base class for all atomic conditions used to setup a scenario. Derived from `py_trees.behaviour.Behaviour`

*All behaviors should use this class as parent*

<details>
<summary>Template</summary>

Args:
- name: Name of the atomic condition

``` python
    def __init__(self, name):
        # Default init. Has to be called via super from derived class
        super(AtomicCondition, self).__init__(name)
        self.name = name

    def setup(self, unused_timeout=15):
        # Default setup
        return True

    def initialise(self):
        # Initialise setup

    def terminate(self, new_status):
        # Default terminate. Can be extended in derived class
```
</details>

## InTriggerDistanceToOSCPosition

This class contains the trigger condition for a distance to an OpenSCENARIO position

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor reached the target distance to the openSCENARIO position

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `osc_position` (str): OpenSCENARIO position
- `distance` (float): Trigger distance between the actor and the target location in meters

</details>

## InTimeToArrivalToOSCPosition

This class contains a trigger if an actor arrives within a given time to an OpenSCENARIO position

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor can reach the position within the given time

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `osc_position`: OpenSCENARIO position
- `time` (float): The behavior is successful, if TTA is less than _time_ in seconds

</details>

## StandStill

This class contains a standstill behavior of a scenario

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor does not move

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `duration` (float): Duration of the behavior in seconds

</details>

## RelativeVelocityToOtherActor

Atomic containing a comparison between an actor's velocity
and another actor's one. The behavior returns SUCCESS when the
expected comparison (greater than / less than / equal to) is achieved

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): actor from which the velocity is taken
- `other_actor` (carla.Actor): The actor with the reference velocity
- `speed` (float): Difference of speed between the actors
- `comparison_operator` (operator): operator used to compare the two velocities

</details>

## TriggerVelocity

Atomic containing a comparison between an actor's speed and a reference one.
The behavior returns SUCCESS when the expected comparison (greater than /
less than / equal to) is achieved.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor from which the speed will be taken.
- `name` (str): Name of the atomic
- `target_velocity` (float): velcoity to be compared with the actor's one
- `comparison_operator` (operator): operator used to compare the two velocities

</details>

## TriggerAcceleration

Atomic containing a comparison between an actor's acceleration
and a reference one. The behavior returns SUCCESS when the
expected comparison (greater than / less than / equal to) is achieved

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `name` (str): Name of the condition
- `target_acceleration` (float): Acceleration reference (in m/s^2) on which the success is dependent
- `comparison_operator` (operator): operator used to compare the two acceleration

</details>

## TimeOfDayComparison

Atomic containing a comparison between the current time of day of the simulation
and a given one. The behavior returns SUCCESS when the
expected comparison (greater than / less than / equal to) is achieved

<details>
<summary>Details</summary>

Args:
- `datetime` (datetime): CARLA actor to execute the behavior
- `target_acceleration` (float): Acceleration reference (in m/s^2) on which the success is dependent
- `comparison_operator` (operator): operator used to compare the two acceleration

</details>

## OSCStartEndCondition

This class contains a check if a named story element has started/terminated.

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the named story element starts

Args:
- `element_name` (str): The story element's name attribute
- `element_type` (str): The element type [act,scene,maneuver,event,action]
- `rule` (str): Either START or END

</details>

## InTriggerRegion

This class contains the trigger region (condition) of a scenario

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor reached the target region

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `min_x`, `max_x`, `min_y`, `max_y` (float): bounding box of the trigger region

</details>

## InTriggerDistanceToVehicle

This class contains the trigger distance (condition) between to actors of a scenario

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor reached the target distance to the other actor

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `reference_actor`(carla.Actor): Reference CARLA actor
- `distance` (float): Trigger distance between the two actors in meters
- `comparison_operator` (operator): comparison operator
- `distance_type` (str): "cartesianDistance"
- `freespace` (float): if True distance is calculated between closest boundary points else it will be from center-center

</details>

## InTriggerDistanceToLocation

This class contains the trigger (condition) for a distance to a fixed
location of a scenario

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor reached the target distance to the given location

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `target_location` (carla.Location): Reference location
- `distance` (float): Trigger distance between the actor and the target location in meters

</details>

## InTriggerDistanceToNextIntersection

This class contains the trigger (condition) for a distance to the
next intersection of a scenario

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor reached the target distance to the next intersection

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `distance` (float): Trigger distance between the actor and the next intersection in meters

</details>

## InTriggerDistanceToLocationAlongRoute

Implementation for a behavior that will check if a given actor
is within a given distance to a given location considering a given route

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor reached the target distance
along its route to the given location

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `distance` (float): Trigger distance between the actor and the next intersection in meters
- `route` (?): Route to be checked
- `location` (carla.Location): Location on the route to be checked

</details>

## InTimeToArrivalToLocation

This class contains a check if a actor arrives within a given time
at a given location.

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor can reach the target location within the given time

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `time` (float): The behavior is successful, if TTA is less than _time_ in seconds
- `location` (carla.Location): Location to be checked in this behavior

</details>

## InTimeToArrivalToVehicle

This class contains a check if a actor arrives within a given time
at another actor.

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor can reach the other vehicle within the given time

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `time` (float): The behavior is successful, if TTA is less than _time_ in seconds
- `other_actor` (carla.Actor): Reference actor used in this behavior

</details>

## InTimeToArrivalToVehicleSideLane

This class contains a check if a actor arrives within a given time
at another actor's side lane. Inherits from InTimeToArrivalToLocation

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor can reach the other vehicle within the given time

Args:
- `actor` (carla.Actor): CARLA actor to execute the behavior
- `time` (float): The behavior is successful, if TTA is less than _time_ in seconds
- `cut_in_lane` (?): the lane from where the other_actor will do the cut in
- `other_actor` (carla.Actor): Reference actor used in this behavior

</details>

## WaitUntilInFront

Behavior that support the creation of cut ins. It waits until the actor has passed another actor

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): the one getting in front of the other actor
- `other_actor` (carla.Actor): the reference vehicle that the actor will have to get in front of
- `factor` (int): How much in front the actor will have to get (from 0 to infinity):
    0: They are right next to each other
    1: The front of the other_actor and the back of the actor are right next to each other

</details>

## DriveDistance

This class contains an atomic behavior to drive a certain distance.

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor drove at least the given distance

Important parameters:
- `actor` (carla.Actor): CARLA actor to execute the condition
- `distance` (float): Distance for this condition in meters

</details>

## AtRightmostLane

This class contains an atomic behavior to check if the actor is at the rightest driving lane.

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the actor enters the rightest lane

Args:
- `actor` (carla.Actor): CARLA actor to execute the condition

</details>

## WaitForTrafficLightState

This class contains an atomic behavior to wait for a given traffic light
to have the desired state.

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the traffic light switches to the desired state

Args:
- `actor` (carla.TrafficLight): CARLA traffic light to execute the condition
- `state` (carla.TrafficLightState): State to be checked in this condition

</details>

## WaitEndIntersection

Atomic behavior that waits until the vehicles has gone outside the junction.
If currently inside no intersection, it will wait until one is found

## WaitForBlackboardVariable

Atomic behavior that keeps running until the blackboard variable is set to the corresponding value.
Used to avoid returning FAILURE if the blackboard comparison fails.

<details>
<summary>Details</summary>

It also initially sets the variable to a given value, if given

Args:
- `variable_name` (str): variable name
- `variable_value` (*): variable value

</details>

## CheckParameter

Atomic behavior that keeps checking global osc parameter value with the given value.

<details>
<summary>Details</summary>

The condition terminates with SUCCESS, when the `comparison_operator` is evaluated successfully.

It also initially sets the variable to a given value, if given

Args:
- `parameter_ref` (object): parameter
- `value` (*): parameter's value
- `comparison_operator` (operator): operator to apply

</details>

---

# Criteria

## Criterion

Base class for all criteria used to evaluate a scenario for success/failure. Derived from `py_trees.behaviour.Behaviour`

<details>
<summary>Template</summary>

Args:
- `expected_value_success`: Result in case of success (e.g. max_speed, zero collisions, ...)
- `expected_value_acceptable`: Result that does not mean a failure, but is not good enough for a success
- `actual_value`: Actual result after running the scenario
- `test_status`: Used to access the result of the criterion
- `optional` (bool) [optional]: Indicates if a criterion is optional (if True, the result is not considered for an overall pass/fail result)

</details>

## MaxVelocityTest

This class contains an atomic test for maximum velocity.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `max_velocity_allowed` (float): maximum allowed velocity in m/s

</details>

## DrivenDistanceTest

This class contains an atomic test to check the driven distance

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `distance_success` (float): If the actor's driven distance is more than this value (in meters), the test result is SUCCESS
- `distance_acceptable` (float): If the actor's driven distance is more than this value (in meters), the test result is ACCEPTABLE

</details>

## AverageVelocityTest

This class contains an atomic test for average velocity.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `avg_velocity_success` (float): If the actor's average velocity is more than this value (in m/s), the test result is SUCCESS
- `avg_velocity_acceptable` (float): If the actor's average velocity is more than this value (in m/s), the test result is ACCEPTABLE

</details>

## CollisionTest

This class contains an atomic test for collisions.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `other_actor` (carla.Actor): only collisions with this actor will be registered
- `other_actor_type` (str): only collisions with actors including this type_id will count.
    Additionally, the "miscellaneous" tag can also be used to include all static objects in the scene
- `terminate_on_failure` (bool) [optional]: If True, the complete scenario will terminate upon failure of this test

``` python
MIN_AREA_OF_COLLISION = 3       # If closer than this distance, the collision is ignored
MAX_AREA_OF_COLLISION = 5       # If further than this distance, the area is forgotten
MAX_ID_TIME = 5                 # Amount of time the last collision if is remembered
```

</details>

## ActorSpeedAboveThresholdTest

This test will fail if the actor has had its linear velocity lower than a specific value for
a specific amount of time

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `speed_threshold` (float): speed required
- `below_threshold_max_time` (float): Maximum time (in seconds) the actor can remain under the speed threshold
- `terminate_on_failure` (bool) [optional]: If True, the complete scenario will terminate upon failure of this test

</details>

## KeepLaneTest

This class contains an atomic test for keeping lane.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test

</details>

## ReachedRegionTest

This class contains the reached region test
The test is a success if the actor reaches a specified region

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `min_x`, `max_x`, `min_y`, `max_y` (float): Bounding box of the checked region

</details>

## OffRoadTest

Atomic containing a test to detect when an actor deviates from the driving lanes. This atomic can
fail when actor has spent a specific time outside driving lanes (defined by OpenDRIVE). Simplified
version of OnSidewalkTest, and doesn't relly on waypoints with *Sidewalk* lane types

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `duration` (float): Time spent at sidewalks before the atomic fails. If terminate_on_failure isn't active, this is ignored.
- `terminate_on_failure` (bool): If True, the atomic will fail when the duration condition has been met.

</details>

## EndofRoadTest

Atomic containing a test to detect when an actor has changed to a different road

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `duration` (float): Time spent after ending the road before the atomic fails. If `terminate_on_failure` isn't active, this is ignored.
- `terminate_on_failure` (bool): If True, the atomic will fail when the duration condition has been met.

</details>

## OnSidewalkTest

Atomic containing a test to detect sidewalk invasions of a specific actor. This atomic can
fail when actor has spent a specific time outside driving lanes (defined by OpenDRIVE).

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `duration` (float): Time spent at sidewalks before the atomic fails. If `terminate_on_failure` isn't active, this is ignored.
- `terminate_on_failure` (bool): If True, the atomic will fail when the duration condition has been met.

</details>

## OutsideRouteLanesTest

Atomic to detect if the vehicle is either on a sidewalk or at a wrong lane. The distance spent outside
is computed and it is returned as a percentage of the route distance traveled.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.ACtor): CARLA actor to be used for this test
- `route` (list [carla.Location, connection]): series of locations representing the route waypoints

``` python
ALLOWED_OUT_DISTANCE = 1.3          # At least 0.5, due to the mini-shoulder between lanes and sidewalks
MAX_ALLOWED_VEHICLE_ANGLE = 120.0   # Maximum angle between the yaw and waypoint lane
MAX_ALLOWED_WAYPOINT_ANGLE = 150.0  # Maximum change between the yaw-lane angle between frames
WINDOWS_SIZE = 3                    # Amount of additional waypoints checked (in case the first on fails)
```

</details>

## WrongLaneTest

This class contains an atomic test to detect invasions to wrong direction lanes.

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test

``` python
MAX_ALLOWED_ANGLE = 120.0
MAX_ALLOWED_WAYPOINT_ANGLE = 150.0
```

</details>

## InRadiusRegionTest

The test is a success if the actor is within a given radius of a specified region

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `x`, `y`, `radius` (float): Position (x,y) and radius (in meters) used to get the checked region

</details>

## InRouteTest

The test is a success if the actor is never outside route. The actor can go outside of the route
but only for a certain amount of distance

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `route` (list [carla.Location, connection]): Route to be checked
- `offroad_max` (float): Maximum distance (in meters) the actor can deviate from the route
- `offroad_min` (float): Maximum safe distance (in meters). Might eventually cause failure
- `terminate_on_failure` (bool) [optional]: If True, the complete scenario will terminate upon failure of this test

``` python
MAX_ROUTE_PERCENTAGE = 30  # %
WINDOWS_SIZE = 5  # Amount of additional waypoints checked
```

</details>

## RouteCompletionTest

Check at which stage of the route is the actor at each tick

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `route` (list [carla.Location, connection]): Route to be checked
- `terminate_on_failure` (bool) [optional]: If True, the complete scenario will terminate upon failure of this test

``` python
DISTANCE_THRESHOLD = 10.0  # meters
WINDOWS_SIZE = 2
```

</details>

## RunningRedLightTest

Check if an actor is running a red light

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `terminate_on_failure` (bool) [optional]: If True, the complete scenario will terminate upon failure of this test

``` python
DISTANCE_LIGHT = 15  # m
```

</details>

## RunningStopTest

Check if an actor is running a stop sign

<details>
<summary>Details</summary>

Args:
- `actor` (carla.Actor): CARLA actor to be used for this test
- `terminate_on_failure` (bool) [optional]: If True, the complete scenario will terminate upon failure of this test

``` python
PROXIMITY_THRESHOLD = 50.0  # meters
SPEED_THRESHOLD = 0.1
WAYPOINT_STEP = 1.0  # meters
```

</details>
