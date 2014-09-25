#!/usr/bin/env python
import rospy
import actionlib

from ros_pololu_servo.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint


class zenoArms:

     # NOTE FROM MELISSA ON IMPLEMENTATION
    # -------------------------------------
    # I use 'position' values are mapped from [-1.0, 1.0] values inclusive. These assume that there exists a function
    # that will use more specific measurements to actually call position.append
    #
    # Velocities will be calculated after the pts are computed, based on the delay from point to point
    #
    # which will translate these [-1.0, 1.0] values to proper radian values for use by the program. Some of these
    # positions may be inverted. arm_map
    # will take this into account and use a -1.0 directional value in order to swap directions, if necessary.
    #
    # This is being done because we do not yet have real config files.
    #
    # I have used '1.0' on joints to mean they are twisted upward for the shoulders and outward for the lengths of the
    # arms in the sense that the elbows would turn in and their wrists would face up. '-1.0' should mean the shoulders
    # are twisted down/back, and the length of the arms are twisted inward in the sense that a human's elbows would
    # turn out and we'd see the backs of their hands. '1.0' on pitch joints means fully 'clenched', and '-1.0' means
    # fully 'extended' As such, I have nearly ensured that one motor on each side will have to be flipped; mirroring
    # the properly oriented motor.
    #--------------------------------------


    #NEED TO BE HAND-SET
    #---------------------
    #the names I will be using for the motors
    names = ['arm_RSR', 'arm_RSP', 'arm_RER', 'arm_REP', 'arm_RHR', 'arm_RHP',
             'arm_LSR', 'arm_LSP', 'arm_LER', 'arm_LEP', 'arm_LHR', 'arm_LHP',
             'waist_R']

    # NEED TO DO: Enter correct radian bounds for each of the above-listed motors
    minRads = [-2.0, -2.0, -2.0, -2.0, -2.0, -2.0,
               -2.0, -2.0, -2.0, -2.0, -2.0, -2.0,
               -2.0]

    maxRads = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
               2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
               2.0]

    # for quick directional adjustments outside of the config file, if necessary
    adjustDir = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                       1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                       1.0]



    # COMPUTED WHILE RUNNING
    # ------------------------------
    # this is overwritten in init
    # it will be multiplied against the [-1.0, 1.0 range to get the value in radians that positions needs to be passed
    rangeRads = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0]

    # this is overwritten in init
    # it will be added to the radians computed with rangeRads if any offset is needed (if abs(minRads) != abs(maxRads)
    offsetRads = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0]

    # for quick programming purposes, I may often want most of the motors NOT to use. This variable stores motor
    # positions so I can retrieve them later
    storePos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0]

    #a constant: assess, do I want to use the storedPosition for this motor?
    storeConst = 12.0



    def __init__(self):
        # NEED TO DO: may be proper location to do min/maxRad computations

        #helper
        motor_total = len(self.names)

        for i in range(0, motor_total):
            #compute something?
            print("minRad: " + self.minRads[i])
            print("maxRad: " + self.maxRads[i])

            #calculate the offset (if for some strange reason abs(maxrads) and abs(minRads) are not equal,
            ## I want to account for it
            self.offsetRads[i] = self.maxRads[i] - self.minRads[i]

            #calculate the 'range' which I will later use
            self.rangeRads[i] = (self.maxRads[i] + self.minRads[i]) / 2.0


    # takes in itself, the point list, and the positions for each of len(self.names) amount of motors
    def mapMotors(self, pts, positions):

        # iterate through all of the motors
        for i in range (0, len(self.positions)):

            # see if the position is carrying storeConst, which means to use whatever position was stored
            # for the motor last.

            if positions[i] == self.storeConst:
                pts.positions.append(self.storePos[i])

            # otherwise flip the direction of the [-1.0, 1.0] value if necessary, transform it into the radians
            # range, and offset it.
            else:
                pts.positions.append(self.rangeRads * positions[i] * self.adjustDir[i] + self.offsetRads[i])


    def defineWave(self):

        #initialize the joint trajectory
        traj = JointTrajectory()

        #really quickly add all of the names
        for i in range (0, 13):
            traj.joint_names.append(names[i])

        # NEUTRAL POSE
        # Zeno should assume a neutral pose, which should not be too abrupt. This posture should have the arms dropped
        # at the side and should be adjusted. The hands may also be imperfect, because 0.0 does not point outward.
        # dropped at his side, Zeno's hand will be at 0.5 or farther (twisted 'outward'

        # Problems are likely to be found in the Shoulder Roll Joints (arm_RSR and arm_LSR) which ought to be at
        # something like 0.5 or -0.5, but which may need fine tuning.

        # initialize the point
        pts = JointTrajectoryPoint()
        beginPoint = 0.5;
        pts.time_from_start = rospy.Duration(beginPoint)

        arm_map(pts, 0.5, arm_RSR)  #the shoulder roll points the arm down
        arm_map(pts, 1.0, arm_RSP)  #the shoulder hinge is tightly clenched
        arm_map(pts, 0.0, arm_RER)  #the elbow is rolled so the hinge(pitch joint) faces forward
        arm_map(-1.0, arm_REP)     #the elbow hinge is fully extended
        arm_map(pts,0.5, arm_RHR)  #the hand rolls to face the leg
        arm_map(pts,0.0, arm_RHP)  #the fingers are partially closed.

        arm_map(pts,0.5, arm_LSR)  #the shoulder roll points the arm down
        arm_map(pts,1.0, arm_LSP)  #the shoulder hinge is tightly clenched
        arm_map(pts,0.0, arm_LER)  #the elbow is rolled so the hinge(pitch joint) faces forward
        arm_map(pts,-1.0, arm_LEP) #the elbow hinge is fully extended
        arm_map(pts,0.5, arm_LHR) #the hand rolls to face the leg
        arm_map(pts,0.0, arm_LHP) #the fingers are partially closed.

        waist_map(0.0)) #the waist should be neutral

        #velocities should be fine-tuned later. As of yet I am not sure how they match up
        for i in range (0, 13):
            pts.velocities.append(arm_map_vel_from_time(, traj.joint_names.append(names[i]))


        #LIFT HAND and TWIST so that the shoulder of the lifted hand faces forward!
         # initialize the point
        pts = JointTrajectoryPoint()
        beginPoint += 1.0
        pts.time_from_start = rospy.Duration(beginPoint)

        arm_map(pts,0.5, arm_RSR)
        arm_map(pts,1.0, arm_RSP)
        arm_map(pts,0.0, arm_RER)
        arm_map(pts,-1.0, arm_REP)
        arm_map(pts,0.5, arm_RHR)
        arm_map(pts,0.0, arm_RHP)

        arm_map(pts,0.5, arm_LSR)
        arm_map(pts,1.0, arm_LSP)
        arm_map(pts,0.0, arm_LER)
        arm_map(pts,-1.0, arm_LEP)
        arm_map(pts,0.5, arm_LHR)
        arm_map(pts,0.0, arm_LHP)



    for n in range (1,4):


        pts=JointTrajectoryPoint()
        pts.time_from_start=rospy.Duration(0.5)
        )
        0.77)
        pts.velocities.append(1.0)
        pts.velocities.append(1.0)
        traj.points.append(pts)
        pts=JointTrajectoryPoint()
        pts.time_from_start=rospy.Duration(1.0)
        0.77)
        -0.77)
        pts.velocities.append(1.0)
        pts.velocities.append(1.0)
        traj.points.append(pts)
        pub.publish(traj)
        rospy.sleep(3.0)



if __name__ == '__main__':
    rospy.init_node('zenoArms')
    client = actionlib.SimpleActionClient('pololu_trajectory_action_server', pololu_trajectoryAction)
    client.wait_for_server()