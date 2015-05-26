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

def run_experiment(num_trials, args):
    print "received message"
    baxter_limb = args[0]
    gripper = args[1]
    traj = args[2]

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
	# turn on both lights
        ser.write('0') # need to send 0 before any of the light signals to tell the arduino the message type
        ser.write('5')
        time.sleep(1)
        # turn off both lights
        ser.write('0')
        ser.write('0')
        time.sleep(1)

    time.sleep(3)

    for i in range(num_trials.data):
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
        # turn on both lights
        ser.write('0')
        ser.write('5')
        ser.write('1 2 ') # fixation onset time
        rospy.loginfo('sent 2')
        time.sleep(1)
        # turn off both lights
        ser.write('0')
        ser.write('0')

        # randomly pick correct object and selected object
        correct = 3
        selected = random.randint(1,5)
	ser.write('1 203 ') #send which object is the correct 1
	rospy.loginfo('sent 203')
        time.sleep(1)
        
        # set light to go on for one second
	# 0 turns off all lights
	# 5 turns on all lights
	# 1-4 turns on the appropriate light, 6 turns on the last LED
        ser.write('0')
        ser.write(str(correct))#turn on the light
        ser.write('1 5 ') # indicate that the light is on
        rospy.loginfo('sent 5')
        time.sleep(.6) # wait 1s
        ser.write('0')
        ser.write('0')#turn off lights
        
        time.sleep(.75 + random.random()*.5-.25) # wait 1s


        # set Baxter joint positions
        down = [[-.86, .22, -.25, .47, -.04, -.59, .20], [-.72, .29, .67, .27, -.08, -.44, -.48], [-.72, .29, .67, .27, -.08, -.44, -.48], [-.72, .29, .67, .27, -.08, -.44, -.48], [-.72, .29, .67, .27, -.08, -.44, -.48]]
        up = [-.80, .16, .09, .43, -.14, -.61, -.03]
        
        print "send baxter to move"
        if selected == 0:
	    num = 220 + selected # make correct codes for objects 1-5
	    num2 = 230 + selected
            ser.write('1 ' + str(num) + ' ')
            rospy.loginfo('sent ' + str(num))
            time.sleep(.1)
            ser.write('1 ' + str(num2) + ' ')
            rospy.loginfo('sent ' + str(num2))
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
        traj.add_point(down[selected-1], 2.0)
        traj.start()
        ser.write('1 51 ')
        rospy.loginfo('sent 51')

	# the following commented code is in case you want to implement feedback
	# this code is for a different paradigm, NOT this paradigm, modifications will be necessary
        '''switched = False

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
            print "error: ", error
            if error == 1 and not switched:
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
            rospy.loginfo('sent 60')'''

        time.sleep(1)

        traj.stop()
        traj.clear('left')
        traj.add_point(up, 2.0)
        traj.start()
        time.sleep(1) 
            
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
        baxter_limb = baxter_interface.Limb('left')
        gripper = baxter_interface.Gripper('left')
        gripper.calibrate()
        traj = Trajectory('left')
        rospy.Subscriber("command", Int32, run_experiment, callback_args=(baxter_limb, gripper, traj))
   
        print "started main node"
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
