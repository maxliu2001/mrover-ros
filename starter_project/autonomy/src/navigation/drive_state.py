import numpy as np

from context import Context
from drive import get_drive_command
from state import BaseState


class DriveState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            # TODO:
            add_outcomes=["driving_to_point", "reached_point"],
        )

    def evaluate(self, ud):
        target = np.array([5.5, 2.0, 0.0])

        # TODO: get the rover's pose, if it doesn't exist stay in DriveState (with outcome "driving_to_point")
        pose = self.context.rover.get_pose()
        if pose is None:
            return "driving_to_point"
        
        # TODO: get the drive command and completion status based on target and pose
        # (HINT: use get_drive_command(), with completion_thresh set to 0.7 and turn_in_place_thresh set to 0.2)
        drive_command, completed = get_drive_command(pose, target, completion_thresh=0.7, turn_in_place_thresh=0.2)

        # TODO: if we are finished getting to the target, go to TagSeekState (with outcome "reached_point")
        if completed:
            return "reached_point"

        # TODO: send the drive command to the rover
        self.context.rover.send_drive_command(drive_command)

        # TODO: tell smach to stay in the DriveState by returning with outcome "driving_to_point"
        return "driving_to_point"
    
