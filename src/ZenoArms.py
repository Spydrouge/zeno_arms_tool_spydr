#!/usr/bin/env python
import rospy
import actionlib

# NEED TO DO: I want to make sure that this random, which is used to make the wave a little 'sloppy' and unpredictable
# so it seems less robotic, is permissable
import random

from ros_pololu_servo.msg import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class ZenoArms:

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
    #       We are assuming from config.xlsx that 'outward' rotates elbows in, and palms up/out
    #       and 'up' for the shoulder pitch means extended, and 'in' for the elbow means clenched
    #--------------------------------------

    # Will be filled with trajectories by the function's like 'define_wave'
    trajAnims = {}

    #NEED TO BE HAND-SET
    #---------------------
    #the names I will be using for the motors, pre-config file delivery
    #names = ['arm_RSR', 'arm_RSP', 'arm_RER', 'arm_REP', 'arm_RHR', 'arm_RHP',
    #         'arm_LSR', 'arm_LSP', 'arm_LER', 'arm_LEP', 'arm_LHR', 'arm_LHP',
    #         'waist_R']

    #the names I am now using, relative to the config file I was given
    names = ['r_shoulder_roll_joint', 'r_shoulder_pitch_joint', 'r_elbow_roll_joint', 'r_elbow_pitch_joint', 'r_wrist_roll_joint', 'r_hand_grasp',
             'l_shoulder_roll_joint', 'l_shoulder_pitch_joint', 'l_elbow_roll_joint', 'l_elbow_pitch_joint', 'l_wrist_roll_joint', 'l_hand_grasp',
             'torso_joint']

    # Entered correct radian bounds for each of the above-listed motors
    minRads = [-1.7453278, -1.3089958, -0.5209888, -0.2498936, -1.4636599, -0.1524795,
               -0.1692518, 0.0000000, -1.1422547, -0.9961139, -0.4146023, -0.3645632,
               -1.0536627]

    maxRads = [0.0000000, 0.00000000, 1.0498062, 0.7973030, 0.1071351, .3711189,
               1.5760760, 1.3089958, 0.4285403, 0.0510828, 1.1561927, 0.1590352,
               0.6916651]

    # NEED TO DO: make sure this is calibrated
    # for quick directional adjustments outside of the config file, if necessary
    adjustDir = [-1.0, 1.0, -1.0, 1.0, -1.0, -1.0,
                 1.0, -1.0, 1.0, -1.0, 1.0, 1.0,
                 1.0]

    #These are in the config file and we currently do not need them
    # motorIds = [6, 0, 1, 2, 3, 4,
    #             13, 12, 14, 15, 16, 17,
    #             8]
    #
    # minPulses = [992, 992, 996.75, 1398, 992, 992,
    #              992, 992, 992, 1136, 996, 992,
    #              1016.25]
    #
    # maxPulses = [1990, 1745.5, 2000, 1931.25, 2000, 1706.25,
    #              1600, 2000, 2000, 1792, 2000, 1701.5,
    #              1907]


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

        #initialize ros node
        rospy.init_node('ZenoArms')

       #iterate through all of the motors
        for i in range(0, len(self.names)):

            #NEED TO DO: Any additional initializations using proper config files, motor numbers, etc?

            print("ZenoArms believes for motor " + self.names[i] + ", minRad: " + str(self.minRads[i]) + " & maxRad: " + str(self.maxRads[i]))

            #calculate the offset (if for some strange reason abs(maxrads) and abs(minRads) are not equal,
            ## I want to account for it
            self.offsetRads[i] = self.minRads[i]

            #calculate the 'range' which I will later use
            self.rangeRads[i] = (self.maxRads[i] + self.minRads[i]) / 2.0

            #calculate the range for the pulses as well, in case we need them.
            #this should allow us to cast [-1.0, 1.0] values to pulses more easily
            #self.rangePulses[i] = self.maxPulses[i] - self.minPulses[i]


        #call definition functions
        self.define_wave()

       #more ros setup
        self.client = actionlib.SimpleActionClient('pololu_trajectory_action_server', pololu_trajectoryAction)
        self.client.wait_for_server()

        #subscribe to listen on the topic of when zeno needs animations for his arms
        rospy.Subscriber("zeno_pololu_animation", String, self.callback)

    def callback(self, msg):
        print ("ZenoArms.callback ran")
        if msg.data in self.trajAnims:
            print(" and found trajAnims correctly.")
            goal = pololu_trajectoryGoal()
            goal.joint_trajectory = self.trajAnims[msg.data]
            self.client.send_goal(goal)
            self.client.wait_for_result()


    # takes in itself, the point list, and the positions for each of len(self.names) amount of motors
    def map_motors(self, pts, positions):

        print("Listing radian amounts so they can be compared with what the PololuController, etc. reports")
        # iterate through all of the motors
        for i in range(0, len(positions)):

            # see if the position is carrying storeConst, which means to use whatever position was stored
            # for the motor last. (the "-" & "<" are used instead of == because these are floats and shouldn't
            # be compared directly)
            if (positions[i] - self.storeConst < 0.01) and (self.storeConst - positions[i] < 0.01):
                pts.positions.append(self.storePos[i])

            # otherwise flip the direction of the [-1.0, 1.0] value if necessary, transform it into the radians
            # range, and offset it by the correct amount
            else:
                final_rad = (self.rangeRads[i] * positions[i]) * self.adjustDir[i] + self.offsetRads[i]
                pts.positions.append(final_rad)

                print("ZenoArms will attempt to get motor " + self.names[i] + " to " + str(final_rad) + " radians.")

                # do not update the storePos array UNLESS we actually have a pos (heavens forbid we throw in a value
                # like "12.0"...
                self.storePos[i] = positions[i]

            #NEED TO DO: This is a placeholder; actual velocities should be computed later
            pts.velocities.append(0.05)

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
        for i in range(0, len(self.names)):
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

        #here we decide whether to wave 2 to 4 times
        sloppy_wave_count = random.randrange(0, 2) + 2
        for j in range(0, sloppy_wave_count):

            #this should make the wave a little uneven and humanish
            sloppy_values = []
            for k in range(0, 4):
                sloppy_values.append(random.uniform(0.0, 0.1) - 0.05)

            # RAISE LEFT ARM
            # The only motors moving should be the shoulder wheel, the elbow, the wrist, the hand, and the waist
            # Zeno raises the shoulder to max, clenches the elbow to max, rotates the wrist to face the hand out (min),
            # extends the fingers to the best of his ability (min), and attempts to rotate the waist so the left shoulder face forward
            #-------------------------------
            arm_poses = [1.0, 1.0, same, 1.0, -0.75, -1.0,
                         same, same, same, same, same,
                         same]
                         #-0.25]

            time_when += 1.5 + sloppy_values[0]

            self.new_points(traj, arm_poses, time_when)

            # EXTEND ARM THROUGH WAVE
            # Zeno waves by unclenching his shoulder hinge (min) and rotating his wrist to try and keep the hand
            # facing forward

            #helper variable for a sloppily extended shoulder hinge (s_piv) and wrist rotation (s_wri) and elbow
            #hinge (s_elb)
            s_piv = sloppy_values[2] - 0.8
            s_wri = sloppy_values[2] - 0.25
            s_elb = sloppy_values[3] * 2 - 0.5

            arm_poses = [same, s_piv , same, s_elb, s_wri, same,
                         same, same, same, same, same,
                         same]
                         #-s_wri]

            time_when += 1.5 + sloppy_values[1]

            self.new_points(traj, arm_poses, time_when)

        # ASSUME NEUTRAL POSE (This will draw the arm back from the extended pose, not the retracted pose)
        arm_poses = [0.5, 1.0, 0.0, -1.0, 0.5, 0.0,
                     0.5, 1.0, 0.0, -1.0, 0.5, 0.0,
                     0.0]

        time_when += 1.5

        self.new_points(traj, arm_poses, time_when)

        # And now that we are done, add it to our animations
        self.trajAnims['zeno_wave'] = traj



if __name__ == '__main__':
    arms=ZenoArms()
    rospy.spin()
