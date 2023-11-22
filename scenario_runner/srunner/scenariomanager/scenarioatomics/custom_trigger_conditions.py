import carla
import operator

from py_trees.common import (Status as BehaviourStatus)

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import AtomicCondition
from srunner.scenariomanager.timer import GameTime

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import calculate_distance

DISTRACTOR = 'trafficcone01'
DISTRACTOR_LOCATIONS = [
    carla.Location(-341.3, 19.0, 0.2),
    carla.Location(-513.7, 173.2, 0.1),
    carla.Location(-370.7, 416.0, 0.1),
    carla.Location(-39.1, 355.4, 0.1),
    carla.Location(2.3, 76.7, 0.1),
    carla.Location(14.2, -197.7, 0.1),
    carla.Location(175.0, -378.0, 0.1),
    carla.Location(382.6, -179.7, 0.1),
    carla.Location(282.9, 23.1, 2.4),
    carla.Location(-23.3, 19.5, 10.7),

    carla.Location(-272.2, 19.1, 1.6),
    carla.Location(-500.1, 97.7, 0.1),
    carla.Location(-481.3, 342.1, 0.1),
    carla.Location(-212.7, 422.1, 0.1),
    carla.Location(14.9, 239.7, 0.1),
    carla.Location(1.6, -75.8, 0.1),
    carla.Location(39.0, -314.1, 0.1),
    carla.Location(310.7, -352.9, 0.1),
    carla.Location(395.0, -119.1, 0.1),
    carla.Location(222.3, 22.1, 5.8),]

class DistractorSpawner(AtomicCondition):

    """
    This class implemented an endless the trigger (condition) that spaws distractors

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - name: Name of the condition
    - distance: Trigger distance between the actor and the spawning location in meters
    - duration: Distractor existance duration in seconds

    The condition will never be terminated with SUCCESS
    """

    def __init__(self,
                 actor,
                 distance=100,
                 duration=1,
                 comparison_operator=operator.lt,
                 name="DistractorSpawner"):
        """
        Setup trigger distance
        """
        super(DistractorSpawner, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        self._actor = actor
        self._distance = distance
        self._duration = duration
        self._comparison_operator = comparison_operator

        self._world = CarlaDataProvider.get_world()

        self._distractor = None
        self._distractor_index = 0
        self._distractor_creation_time = 0
        self._distractor_location = DISTRACTOR_LOCATIONS[self._distractor_index]

        print(f'DISTRACTOR: distance={self._distance}, duration={self._duration}')

    def update(self):
        """
        Check if the actor is within trigger distance to the spawning location
        """

        if self._distractor is not None:
            if (GameTime.get_time() - self._distractor_creation_time) > self._duration:
                self._remove_distractor()

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
            print(f'DISTRACTOR: {DISTRACTOR} removed at {GameTime.get_time()}')

    def _create_distractor(self):

        self._remove_distractor()

        try:
            transform = carla.Transform(self._distractor_location, carla.Rotation())
            bp = self._world.get_blueprint_library().find(f'static.prop.{DISTRACTOR}')
            distractor = self._world.spawn_actor(bp, transform) if bp is not None else None
            if distractor is not None:
                self._distractor = distractor
                self._distractor_creation_time = GameTime.get_time()
                print(f'DISTRACTOR: {DISTRACTOR} created at {self._distractor_location} / {self._distractor_creation_time}')
        except:
            print(f'DISTRACTOR: Cannot create {DISTRACTOR} at {self._distractor_location}')
        finally:
            self._distractor_index += 1
            if self._distractor_index == len(DISTRACTOR_LOCATIONS):
                self._distractor_index = 0
            self._distractor_location = DISTRACTOR_LOCATIONS[self._distractor_index]
        