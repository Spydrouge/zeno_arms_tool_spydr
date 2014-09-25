#!/usr/bin/env python
import rospy
import actionlib

from ros_pololu_servo.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint


class zenoArms:

    # NOTE ON IMPLEMENTATION
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

    # NOTE ON JOINT ORIENTATION
    # I am using a human sense of joint orientation to make animating easier (ie: -1 means the same thing for each elbow
    # In reality is more than likely that joints on opposite sides of the body are flipped/mirrored, so -1 on one side
    # will be 1 on the other.
    #
    # Assumptions:
    #       Hinge (Pitch) Joints are max at fully clenched and min at fully extended
    #       Wheel (Rotation) Joints are max when:
    #           - For the Shoulders: When the arms are lifted above the head
    #           - For the Elbows: Assume the arms are slack. max will rotate the hard tip of the elbow inward.
    #           - For the Wrists: Same orientation as the elbows. Palms will turn out/up.
    #           - For the Waist: Right shoulder is forward, left shoulder is back
    #
    # Corrections:
    #       Corrections should be easy to make using the adjustDir array, featured below. This will quickly flip the
    #       radians values.

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

    #a constant: assess, do I want to use the storedPosition for this motor? If so, I will pass in storeConst instead of a
    #[-1.0, 1.0] value.
    storeConst = 12.0


    # initialize many of the values we may require in order to correctly transform [-1.0, 1.0] range values
    # into the proper radian ranges, and to compute velocities as well

    def __init__(self):

       #iterate through all of the motors
        for i in range(0, len(self.names)):

            #NEED TO DO: Any additional initializations using proper config files, motor numbers, etc?

            print("minRad: " + self.minRads[i])
            print("maxRad: " + self.maxRads[i])

            #calculate the offset (if for some strange reason abs(maxrads) and abs(minRads) are not equal,
            ## I want to account for it
            self.offsetRads[i] = self.maxRads[i] - self.minRads[i]

            #calculate the 'range' which I will later use
            self.rangeRads[i] = (self.maxRads[i] + self.minRads[i]) / 2.0

    # takes in itself, the point list, and the positions for each of len(self.names) amount of motors
    def map_motors(self, pts, positions):

        # iterate through all of the motors
        for i in range(0, len(positions)):

            # see if the position is carrying storeConst, which means to use whatever position was stored
            # for the motor last.
            if positions[i] == self.storeConst:
                pts.positions.append(self.storePos[i])

            # otherwise flip the direction of the [-1.0, 1.0] value if necessary, transform it into the radians
            # range, and offset it by the correct amount
            else:
                pts.positions.append(self.rangeRads * positions[i] * self.adjustDir[i] + self.offsetRads[i])

                # do not update the storePos array UNLESS we actually have a pos (heavens forbid we throw in a value
                # like "12.0"...
                self.storePos[i] = positions[i]

            #NEED TO DO: This is a placeholder; actual velocities should be computed later
            pts.velocities.append(1.0)

    def new_points(self, traj, positions, begin_point):
        # initialize the point
        pts = JointTrajectoryPoint()

        #Adjust as necessary. A little delay should keep strange abrupt jerks from occurring
        pts.time_from_start = rospy.Duration(begin_point)

        # map_motors will do all our transformations from the [-1.0, 1.0] system to what the motors actually need,
        # and append position values to pts
        self.map_motors(pts, positions)

        # we are appending the points to the overall trajectory variable right now. Later, an array will go back
        # and give the points proper velocities.
        traj.points.append(pts)

    def define_wave(self):

        #initialize the joint trajectory
        traj = JointTrajectory()

        #really quickly add all of the motor names
        for i in range (0, len(self.names)):
            traj.joint_names.append(self.names[i])


        #another lovely helping variable that allows us to pass in the storeConst with the word 'same' when we want
        #positions to stay the same
        same = self.storeConst

        #this variable will help us keep track of how much time has elapsed, so we don't accidentlaly put one trajectory
        #behind another
        time_when = 0.0

        # NEUTRAL POSE
        # ----------------------
        # Zeno should 'gently' assume a neutral position to wave from. This posture should have the arms dropped
        # at the side with the hands pointed towards the thigh.

        # Problems are likely to be found in the Shoulder Roll Joints (arm_RSR and arm_LSR) which ought to be at
        # something like 0.5 or -0.5, and the hands, which do not appear to be attached so that 0.0 really lines
        # up with anything...


        # This helper variable should be easy to edit. It lists the motor positions, [-1.0, 1.0] and exists to
        # conveniently pass them into map_motors for each new position. arm_poses will pass in storeConst from now on
        # if it wants to use the last position array.
        arm_poses = [0.5, 1.0, 0.0, -1.0, 0.5, 0.0,
                     0.5, 1.0, 0.0, -1.0, 0.5, 0.0,
                     0.0]

        time_when += 0.5

        # and now of course call the function
        self.new_points(traj, arm_poses, time_when)

        # RAISE LEFT ARM
        # The only motors moving should be the shoulder wheel, the elbow, the wrist, the hand, and the waist
        # Zeno raises the shoulder to max, clenches the elbow to max, rotates the wrist to face the hand out (min),
        # extends the fingers to the best of his ability (min), and attempts to rotate the waist so the left shoulder face forward
        #-------------------------------
        arm_poses = [same, same, same, same, same,
                     1.0, same, same, 1.0, -1.0, same,
                     -1.0]

        time_when += 0.75

        self.new_points(traj, arm_poses, time_when)

        #





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