import carla
import operator

from typing import cast

from py_trees.common import (Status as BehaviourStatus)

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import AtomicCondition
from srunner.scenariomanager.timer import GameTime

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import calculate_distance
from srunner.scenariomanager.scenarioatomics.custom_behaviors import (print_debug, EMirrorsScoresClient)

class DistractorSpawner(AtomicCondition):

    """
    This class implementes the trigger (condition) that spaws distractors

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - type: distractor type, should be XXX in CARLA's static.prop.XXX
    - location: list of carla.Location for the distractor to appear
    - name: Name of the condition
    - distance: Trigger distance between the actor and the spawning location in meters
    - duration: Distractor existance duration in seconds
    - loop: if 'True' (default), then the spawning restarts from the beginning of the list of locations upon reaching the end

    The condition will be terminated with SUCCESS if loop=False after the distractor is removed from the last location,
    otherwise the 'update' function always returns RUNNING
    """

    def __init__(self,
                 actor,
                 type,
                 locations,
                 distance=100.0,
                 duration=1.0,
                 loop=True,
                 comparison_operator=operator.lt,
                 name="DistractorSpawner"):
        """
        Setup trigger distance
        """
        super(DistractorSpawner, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        self._actor = actor
        self._distractor_type = f'static.prop.{type}'
        self._distractor_locations = locations
        self._distance = distance
        self._duration = duration
        self._comparison_operator = comparison_operator
        self._loop = loop

        self._world = cast(carla.World, CarlaDataProvider.get_world())

        self._distractor = None
        self._distractor_index = 0
        self._distractor_creation_time = 0
        self._distractor_location = self._distractor_locations[self._distractor_index]

        print(f'DISTRACTOR: distance={self._distance}, duration={self._duration}')

    def update(self):
        """
        Check if the actor is within trigger distance to the spawning location
        """

        if self._distractor is not None:
            if (GameTime.get_time() - self._distractor_creation_time) > self._duration:
                self._remove_distractor()
                if not self._loop and self._distractor_index == 0:
                    return BehaviourStatus.SUCCESS

        location = CarlaDataProvider.get_location(self._actor)

        if location is not None:
            if self._comparison_operator(calculate_distance(location, self._distractor_location), self._distance):
                self._create_distractor()

        return BehaviourStatus.RUNNING

    def terminate(self, new_status):
        super(DistractorSpawner, self).terminate(new_status)
        self._remove_distractor()

    def _remove_distractor(self):
        if (self._distractor is not None):
            self._distractor.destroy()
            self._distractor = None

            EMirrorsScoresClient.send(f'distractor\thide')
            print_debug(f'DISTRACTOR: {self._distractor_type} removed / {GameTime.get_time():.1f} sec')

    def _create_distractor(self):

        self._remove_distractor()

        x, y = self._distractor_location.x, self._distractor_location.y
        time = self._distractor_creation_time

        try:
            transform = carla.Transform(self._distractor_location, carla.Rotation())
            bp = self._world.get_blueprint_library().find(self._distractor_type)
            distractor = self._world.spawn_actor(bp, transform) if bp is not None else None
            if distractor is not None:
                self._distractor = distractor
                self._distractor_creation_time = GameTime.get_time()

                EMirrorsScoresClient.send(f'distractor\tshow\t{x:.1f},{y:.1f}')
                print_debug(f'DISTRACTOR: {self._distractor_type} created at ({x:.1f}, {y:.1f}) / {time:.1f} sec')
        except:
            print_debug(f'DISTRACTOR: Cannot create {self._distractor_type} at ({x:.1f}, {y:.1f})')
        finally:
            self._distractor_index += 1
            if self._distractor_index == len(self._distractor_locations):
                self._distractor_index = 0
            self._distractor_location = self._distractor_locations[self._distractor_index]
        