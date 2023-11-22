import carla
import operator

from py_trees.common import (Status as BehaviourStatus)

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import AtomicCondition
from srunner.scenariomanager.timer import GameTime

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import calculate_distance

DISTRACTOR = 'trafficcone01'
DISTRACTOR_LOCATIONS = [
    carla.Location(-282.1, 6.1, 1.3),
    carla.Location(-499.9, 93.2, 0.1),
    carla.Location(-460.0, 375.8, 0.1),
    carla.Location(-195.9, 422.0, 0.1),
    carla.Location(1.7, 242.6, 0.1),
    carla.Location(14.8, -39.0, 0.1),
    carla.Location(30.8, -287.2, 0.1),
    carla.Location(315.2, -362.6, 0.1),
    carla.Location(382.0, -120.1, 0.1),
    carla.Location(156.2, 8.0, 9.2),

    carla.Location(-276.9, 19.1, 1.4),
    carla.Location(-500.6, 149.6, 0.1),
    carla.Location(-365.3, 429.9, 0.1),
    carla.Location(-94.0, 392.8, 0.1),
    carla.Location(15.8, 179.7, 0.1),
    carla.Location(1.3, -149.3, 0.1),
    carla.Location(179.7, -365.0, 0.1),
    carla.Location(383.0, -221.1, 0.1),
    carla.Location(350.6, 9.7, 0.4),
    carla.Location(71.0, 19.9, 11.1),
    carla.Location(222.3, 22.1, 5.8),
]

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
        