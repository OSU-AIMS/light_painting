#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: A.C. Buynak


class SimpleMoverClient(object):
    """SimpleMover Class"""

    def __init__(self):

        # Imports
        import actionlib
        from light_painting.msg import SimpleMoveRequestAction

        # Setup Action Client
        self.c = actionlib.SimpleActionClient('/move_descartes_line', SimpleMoveRequestAction)
        self.c.wait_for_server()

    def setNewGoal(self, newPose, dt):
        '''
        Send pose action request to the SimpleMover action server.

        :return: Success of action [bool]
        '''

        # Setup Empty Goal
        import rospy
        from light_painting.msg import SimpleMoveRequestGoal
        req = SimpleMoveRequestGoal()

        # Set inputs
        req.goal = newPose
        req.duration = dt

        # Send goal
        self.c.send_goal(req)

        # Check & Report
        tsec = 10
        if self.c.wait_for_result(rospy.Duration(tsec,0)) != True:
            rospy.logwarn('SimpleMover did not achieve goal within %i seconds.', tsec)
            return False
        
        return self.c.get_result().complete



# if __name__ == '__main__':
#     exit(f'{__file__} is not meant to be run independently.')



if __name__ == '__main__':

    import rospy
    rospy.loginfo("THIS SCRIPT TO BE RUN ONLY FOR UNIT TESTING HARDWARE:")


    # Setup
    rospy.init_node('test')
    sm = SimpleMoverClient()

    rospy.loginfo("Setup complete.")

    # Build Pose Goal
    from geometry_msgs.msg import Pose
    goal = Pose()
    goal.position.x = 0.5
    goal.position.y = 0
    goal.position.z = 0.8
    goal.orientation.x = 0.707
    goal.orientation.y = 0
    goal.orientation.z = 0.707
    goal.orientation.w = 0

    # Send Pose Goal
    result = sm.setNewGoal(goal, 1)
    rospy.loginfo("Result: %s", str(result))

    rospy.loginfo("Done")

