from enum import Enum, auto

from typing import Callable, Tuple
from commands2.button import Trigger
from ntcore import NetworkTableInstance
from wpilib import Joystick


Axis = Callable[[], float]


class AxisButton(Trigger):
    """a trigger that can be fired by an axis hitting a certain limit"""

    def __init__(self, axis: Axis, threshold: float) -> None:
        self.axis = axis
        self.threshold = threshold
        super().__init__(lambda: self.axis() > self.threshold)


class NetworkTableButton(Trigger):
    def __init__(self, key: str) -> None:
        super().__init__(
            NetworkTableInstance.getDefault().getBooleanTopic(key).subscribe(False).get
        )


class ModifiableJoystickButton(Trigger):
    def _active(self):
        joystick, button = self.call()
        return joystick.getRawButton(button)

    def __init__(self, call: Callable[[], Tuple[Joystick, int]]):
        self.call = call
        super().__init__(self._active)


class DPadButton(Trigger):
    """a trigger that can be fired by a d pad button"""

    class DPad(Enum):
        Up = auto()
        Down = auto()
        Left = auto()
        Right = auto()

        def isNum(self, direction: int):
            if self == self.Up:
                return direction in [315, 0, 45]
            elif self == self.Down:
                return 135 <= direction <= 225
            elif self == self.Left:
                return 225 <= direction <= 315
            elif self == self.Right:
                return 45 <= direction <= 135
            else:
                return False

    def __init__(self, controller: Joystick, POVNumber: int, button: DPad):
        super().__init__(lambda: button.isNum(controller.getPOV(POVNumber)))
