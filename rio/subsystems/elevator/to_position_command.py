from commands2 import Command

from subsystems.elevator.elevator import Elevator


class ToPositionCommand(Command):
    def __init__(self, elevator: Elevator, postion):
        super().__init__()
        self.elevator = elevator
        self.targetPosition = postion
        self.feedForward = 6.0
        self.addRequirements(elevator)

    def initialize(self) -> None:
        self.feedForward = 6.0
        if self.elevator.getPosition() >= self.targetPosition:
            self.feedForward = 0.0
    
    def execute(self) -> None:
        self.elevator.move_to_position_with_ff(self.targetPosition, self.feedForward)
