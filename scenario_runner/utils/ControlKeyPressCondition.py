import py_trees

from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import AtomicCondition

from pygame.locals import (K_UP, K_DOWN, K_LEFT, K_RIGHT, K_w, K_a, K_s, K_d)

from net.tcp_client import TcpClient


class ControlKeyPressCondition(AtomicCondition):

    """
    Connects to the server that is running in manual_control.py
    and awaits for the control key press

    Important parameters:
    - name: Name of the atomic condition
    """

    def __init__(self, name):
        """
        Setup parameters
        """
        super(ControlKeyPressCondition, self).__init__(name)
        self._last_pressed_key = 0

    def initialise(self):
        """
        TCP client initialization
        """
        self._tcp_client = TcpClient("localhost")
        self._tcp_client.connect(self._handle_net_request)

        super(ControlKeyPressCondition, self).initialise()

    def terminate(self, new_status):
        """
        TCP client termination
        """
        if new_status == py_trees.common.Status.SUCCESS:
            self._tcp_client.close()
            
        super(ControlKeyPressCondition, self).terminate(new_status)

    def update(self):
        """
        Check if a control key was pressed
        """
        new_status = py_trees.common.Status.RUNNING

        if self._last_pressed_key in (K_UP, K_DOWN, K_LEFT, K_RIGHT, K_w, K_a, K_s, K_d):
            new_status = py_trees.common.Status.SUCCESS
            
        return new_status
    
    def _handle_net_request(self, req: str) -> None:
        try:
            self._last_pressed_key = int(req)
        except:
            print(f'CHANGE LANE [CKPC]: net request failed: {req}')

