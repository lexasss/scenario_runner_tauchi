import py_trees
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (Idle)

class WaitForEvent(Idle):

    """
    Keeps running until the variable is set globally

    Important parameters:
    - timeout: if set, then triggers the event after the timeout,
               otherwise waits until someone else triggers the event
    - name: Name of the atomic condition
    """

    is_set = False      # global variable to be set to trigger the event

    def __init__(self, timeout=float("inf"), name="RunUntilVariableIsSet"):
        """
        Setup parameters
        """
        super(WaitForEvent, self).__init__(duration=timeout, name=name)

    def update(self):
        """
        Check if a control key was pressed
        """
        new_status = super(WaitForEvent, self).update()
        if new_status == py_trees.common.Status.SUCCESS:
           WaitForEvent.is_set = True 

        if WaitForEvent.is_set:
            new_status = py_trees.common.Status.SUCCESS
            
        return new_status
