#!/usr/bin/env python

import rospy
import random
from copy import copy
import baxter_interface
from std_msgs.msg import Int32
import serial
import time
import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

# communicates to EEG-reading computer
ser = serial.Serial("/dev/ttyACM2", 9600, timeout=1)

def run_experiment(num_trials, arduino_pub, baxter_limb, gripper, pub, traj):

    ser.write('1 111 ') # need to send 1 before any of the EEG outputs to tell the arduino what type of message it is # send 111 three times to signal start of experiment
    rospy.loginfo('sent 111')
    time.sleep(.1)
    ser.write('1 111 ')
    rospy.loginfo('sent 111')
    time.sleep(.1)
    ser.write('1 111 ')
    rospy.loginfo('sent 111')
    time.sleep(.1)
    print "sent 111 three times"

    # set arduinos to blink 3 times
    for i in range(3):
        #arduino_pub.publish(4) # turn on both lights
        ser.write('0') # need to send 0 before any of the light signals to tell the arduino the message type
        ser.write('5')
        time.sleep(1)
        #arduino_pub.publish(5) # turn off both lights
        ser.write('0')
        ser.write('0')
        time.sleep(1)

    time.sleep(3)

    for i in range(num_trials):
        print "starting trial"
        ser.write('1 90 ') # indicate start of trial
        rospy.loginfo('sent 90')
        time.sleep(.1)
        ser.write('1 104 ') # Baxter's task ID
        rospy.loginfo('sent 104')
        time.sleep(.1)
        ser.write('1 161 ') # blocks, each with a different paradigm???
        rospy.loginfo('sent 161')
        time.sleep(.1)
        ser.write('1 ' + str(170 + (i%30)) + ' ') # trial number, going 170-199 then repeating
        rospy.loginfo('sent '+ str(170 + (i%30)))
        time.sleep(.1)
        ser.write('1 141 ') # training (closed loop should be 142)
        rospy.loginfo('sent 141')
        time.sleep(.1)
        ser.write('1 151 ') # for training, testing should be 152
        rospy.loginfo('sent 151')
        time.sleep(.1)
        ser.write('1 1 ') # inter trial interval
        rospy.loginfo('sent 1')
        time.sleep(.1)

        # set both LEDs to blink once
        #arduino_pub.publish(4) # turn on both lights
        ser.write('0')
        ser.write('5')
        ser.write('1 2 ') # fixation onset time
        rospy.loginfo('sent 2')
        time.sleep(1)
        #arduino_pub.publish(5) # turn off both lights
        ser.write('0')
        ser.write('0')

        # randomly pick correct object and selected object
        correct = random.randint(0,1)
        selected = random.randint(0,1)
        if correct == 0:
            ser.write('1 201')
            rospy.loginfo('sent 201')
        else:
            ser.write('1 202 ')
            rospy.loginfo('sent 202')
        time.sleep(1)

        # set light to go on for one second
        # 0 -- turn on light 0
        # 1 -- turn on light 1
        # 2 -- turn off light 0
        # 3 -- turn off light 1
        ser.write('0')
        ser.write(str(correct+1))#turn on the light
        ser.write('1 5 ') # indicate that the light is on
        rospy.loginfo('sent 5')
        time.sleep(.6) # wait 1s
        ser.write('0')
        ser.write('0')#turn off lights

        time.sleep(.75 + random.random()*.5-.25) # wait 1s


        # set Baxter joint positions
        #down = [[-1.19, .41, .28, .48, -.79, -.70, .14], [-.56, .33, .13, .62, .17, -.78, -.25]]
        down = [[-.86, .22, -.25, .47, -.04, -.59, .20], [-.72, .29, .67, .27, -.08, -.44, -.48]]
        up = [-.80, .16, .09, .43, -.14, -.61, -.03]
        wait = [-.75, -.21, -.07, 1.84, .05, -1.57, .04]

        print "send baxter to move"
        if selected == 0:
            ser.write('1 221 ') # left target
            rospy.loginfo('sent 221')
            time.sleep(.1)
            ser.write('1 231 ') # left target
            rospy.loginfo('sent 231')
            time.sleep(.1)
        else:
            ser.write('1 222 ') # right target (was 222)
            rospy.loginfo('sent 222')
            time.sleep(.1)
            ser.write('1 232 ') # right target (was 232)
            rospy.loginfo('sent 232')
            time.sleep(.1)
        if correct == selected:
            ser.write('1 25 ') # reward
            rospy.loginfo('sent 25')
            time.sleep(.1)
            ser.write('1 26 ') # reward
            rospy.loginfo('sent 26')
            time.sleep(.1)
        else:
            ser.write('1 11 ') # punisher
            rospy.loginfo('sent 11')
            time.sleep(.1)
            ser.write('1 21 ') # punisher
            rospy.loginfo('sent 21')
            time.sleep(.1)

        # have Baxter go to pick up objects
        traj.stop()
        traj.clear('left')
        traj.add_point(down[selected], 2.0)
        traj.start()
        ser.write('1 51 ')
        rospy.loginfo('sent 51')
        switched = False

        for i in range(20):
            #time.sleep(.06)
            #ser.write('1 51 ')
            time.sleep(.1)
            ser.write('2')  # tell arduino to respond with the input from the EEG
            error = ser.read() # read one byte -- should be sufficient for reading in the 0/1 message
            try:
                error = int(error.encode('hex'))
            except:
                error = 0
		print "HEX reading error"
                rospy.loginfo("HEX reading error")
            print "error: ", error
            if error == 1 and not switched:
                rospy.loginfo("Got 1 signal!")
                selected = 1 - selected
                if selected == 0:
                    ser.write('1 61 ') # new target is left
                    rospy.loginfo('sent 61')
                else:
                    ser.write('1 62 ') # new target is right
                    rospy.loginfo('sent 62')
                print "Baxter switching!"
                switched = True
                traj.stop()
                traj.clear('left')
                traj.add_point(down[selected], 2.0)
                traj.start()

        if not switched:
            ser.write('1 60 ') # target unchanged
            rospy.loginfo('sent 60')

        time.sleep(1)

        ### read from serial port to see if there's feedback, change behavior if necessary ###
        # send code to trigger response
        #ser.write('1 51 ') # indicates whether input was received
        #ser.write('2') # tell arduino to respond with the input from the EEG
        #error = ser.read() # read one byte -- should be sufficient for reading in the 0/1 message
        #if error == 1: # if there was an error
        #    selected = 1 - selected # then switch which target Baxter selects
        #    baxter_limb.move_to_joint_positions(wait) # go back to the wait position
        #    time.sleep(4)
        #    baxter_limb.move_to_joint_positions(down[selected]) # go to the new, correct location
        #    time.sleep(4)
        #    ### end response to serial data section ###

        traj.stop()
        traj.clear('left')
        traj.add_point(up, 2.0)
        traj.start()
        time.sleep(1) # give Baxter 4 seconds to get there

        ser.write('1 91 ') # trial end
        rospy.loginfo('sent 91')
        time.sleep(.1)

    ser.write('1 100 ') # indicate end of session
    rospy.loginfo('sent 100')

    # set arduinos to blink 3 times
    for i in range(3):
        #arduino_pub.publish(4) # turn on both lights
        ser.write('0') # need to send 0 before any of the light signals to tell the arduino the message type
        ser.write('5')
        time.sleep(1)
        #arduino_pub.publish(5) # turn off both lights
        ser.write('0')
        ser.write('0')
        time.sleep(1)

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        else:
            print "no time out -- ready to go!"
        self.clear(limb)

    def add_point(self, positions, time):
        print "add!"
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

if __name__ == '__main__':
    try:
        rospy.init_node('feedback_control', anonymous=True)
        arduino_pub = rospy.Publisher("arduino_light", Int32)
        baxter_limb = baxter_interface.Limb('left')
        gripper = baxter_interface.Gripper('left')
        gripper.calibrate()
        pub = rospy.Publisher("eeg_output", Int32)
        traj = Trajectory('left')
        rate = rospy.Rate(10) # 10hz

        st = raw_input("Enter subject name: ")
        rospy.loginfo("Subject: " + st)

        while not rospy.is_shutdown():
            st = raw_input("Enter number of trials: ")
            try:
                num_trials = int(st)
                rospy.loginfo("Number of trials: " + str(num_trials))
                st = raw_input("Enter block name: ")
                rospy.loginfo("block name: " + st)
                run_experiment(num_trials, arduino_pub, baxter_limb, gripper, pub, traj)
            except ValueError:
                pass
            rate.sleep()
        #rospy.Subscriber("command", Int32, run_experiment, callback_args=(arduino_pub, baxter_limb, gripper, pub, traj))

    except rospy.ROSInterruptException:
        pass
