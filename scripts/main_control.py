#!/usr/bin/env python

import rospy
import random
import baxter_interface
from std_msgs.msg import Int32
import serial
import time

# communicates to EEG-reading computer
ser = serial.Serial("/dev/ttyACM2", 9600, timeout=1)

def run_experiment(num_trials, args):
    print "received message"
    arduino_pub = args[0]
    baxter_limb = args[1]
    gripper = args[2]
    pub = args[3]

    ser.write('1 111 ') # need to send 1 before any of the EEG outputs to tell the arduino what type of message it is # send 111 three times to signal start of experiment
    time.sleep(.1)
    ser.write('1 111 ')
    time.sleep(.1)
    ser.write('1 111 ')
    time.sleep(.1)
    print "sent 111 three times"

    # set arduinos to blink 3 times
    for i in range(3):
        #arduino_pub.publish(4) # turn on both lights
        ser.write('0') # need to send 0 before any of the light signals to tell the arduino the message type
        ser.write('4')
        time.sleep(1)
        #arduino_pub.publish(5) # turn off both lights
        ser.write('0')
        ser.write('5')
        time.sleep(1)

    time.sleep(3)

    for i in range(num_trials.data):
        print "starting trial"
        ser.write('1 90 ') # indicate start of trial
        time.sleep(.1)
        ser.write('1 104 ') # Baxter's task ID
        time.sleep(.1)
        ser.write('1 161 ') # blocks, each with a different paradigm???
        time.sleep(.1)
        ser.write('1 ' + str(170 + (i%30)) + ' ') # trial number, going 170-199 then repeating
        time.sleep(.1)
        ser.write('1 141 ') # training (closed loop should be 142)
        time.sleep(.1)
        ser.write('1 151 ') # for training, testing should be 152
        time.sleep(.1)
        ser.write('1 1 ') # inter trial interval
        time.sleep(.1)

        # set both LEDs to blink once
        #arduino_pub.publish(4) # turn on both lights
        ser.write('0')
        ser.write('4')
        ser.write('1 2 ') # fixation onset time
        time.sleep(1)
        #arduino_pub.publish(5) # turn off both lights
        ser.write('0')
        ser.write('5')

        # randomly pick correct object and selected object
        correct = random.randint(0,1)
        selected = random.randint(0,1)
        if correct == 0:
            ser.write('1 201')
        else:
            ser.write('1 202 ')
        time.sleep(1)
        
        # set light to go on for one second
        # 0 -- turn on light 0
        # 1 -- turn on light 1
        # 2 -- turn off light 0
        # 3 -- turn off light 1
        ser.write('0')
        ser.write(str(correct))#turn on the light
        ser.write('1 5 ') # indicate that the light is on
        time.sleep(.6) # wait 1s
        ser.write('0')
        ser.write(str(correct+2))#turn off the light
        
        time.sleep(.75 + random.random()*.5-.25) # wait 1s


        # set Baxter joint positions
        down = [{'left_w0': -0.794602047216797, 'left_w1': -0.7033301904418946, 'left_w2': 0.13882526114501953, 'left_e0': 0.27726702709350587, 'left_e1': 0.4797524908630371, 'left_s0': -1.1919030708251954, 'left_s1': 0.41685927863159183}, {'left_w0': 0.16758740088500979, 'left_w1': -0.7838641817138673, 'left_w2': -0.25118935372924805, 'left_e0': 0.12847089083862306, 'left_e1': 0.6181942568115235, 'left_s0': -0.5602864821350098, 'left_s1': 0.33364082098388675}] # [location of object 0, location of object 1]
        up = {'left_w0': -0.14419419389648439, 'left_w1': -0.6093738672912598, 'left_w2': -0.027228158953857422, 'left_e0': 0.09357282795410157, 'left_e1': 0.43334957208251956, 'left_s0': -0.8026554463439942, 'left_s1': 0.1606844873474121} # runs after 3
#{'left_w0': -0.15148060263061525, 'left_w1': -0.8548107930725098, 'left_w2': -0.024160197381591798, 'left_e0': 0.19059711267700197, 'left_e1': 0.5200194864990235, 'left_s0': -0.8479078795349122, 'left_s1': 0.12770390044555666} # picked up location # runs number one and two: 001/20150412/1,2
        wait = {'left_w0': 0.04793689956665039, 'left_w1': -1.5715633153930666, 'left_w2': 0.042951462011718754, 'left_e0': -0.06672816419677735, 'left_e1': 1.8396264577697754, 'left_s0': -0.7478156332397461, 'left_s1': -0.2059369205383301} # waiting location

        print "send baxter to move"
        if selected == 0:
            ser.write('1 221 ') # left target
            time.sleep(.1)
            ser.write('1 231 ') # left target
            time.sleep(.1)
        else:
            ser.write('1 222') # right target
            time.sleep(.1)
            ser.write('1 232') # right target
            time.sleep(.1)
        if correct == selected:
            ser.write('1 25 ') # reward
            time.sleep(.1)
            ser.write('1 26 ') # reward
            time.sleep(.1)
        else:
            ser.write('1 11 ') # punisher
            time.sleep(.1)
            ser.write('1 21 ') # punisher
            time.sleep(.1)

        # have Baxter go to pick up objects
        baxter_limb.move_to_joint_positions(down[selected]) # go to object location
        time.sleep(4) # give Baxter 4 seconds to get there

        ### read from serial port to see if there's feedback, change behavior if necessary ###
        # send code to trigger response
        ser.write('1 51 ') # indicates whether input was received
        #ser.write('2') # tell arduino to respond with the input from the EEG
        #error = ser.read() # read one byte -- should be sufficient for reading in the 0/1 message
        #if error == 1: # if there was an error
        #    selected = 1 - selected # then switch which target Baxter selects
        #    baxter_limb.move_to_joint_positions(wait) # go back to the wait position
        #    time.sleep(4)
        #    baxter_limb.move_to_joint_positions(down[selected]) # go to the new, correct location
        #    time.sleep(4)
        #    ### end response to serial data section ###

        #gripper.close()
        #time.sleep(2) # let the gripper have time to close
        #baxter_limb.move_to_joint_positions(up)
        #time.sleep(4) # give Baxter 4 seconds to get there
        #baxter_limb.move_to_joint_positions(down[selected])
        #time.sleep(4) # give Baxter 4 seconds to get there
        #gripper.open()
        #time.sleep(2) # let the gripper have time to open
        baxter_limb.move_to_joint_positions(up)
        time.sleep(4) # give Baxter 4 seconds to get there
            
        ser.write('1 91 ') # trial end
        time.sleep(.1)
        
    ser.write('1 100 ') # indicate end of session

    # set arduinos to blink 3 times
    for i in range(3):
        #arduino_pub.publish(4) # turn on both lights
        ser.write('0') # need to send 0 before any of the light signals to tell the arduino the message type
        ser.write('4')
        time.sleep(1)
        #arduino_pub.publish(5) # turn off both lights
        ser.write('0')
        ser.write('5')
        time.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('main_control', anonymous=True)
        arduino_pub = rospy.Publisher("arduino_light", Int32)
        baxter_limb = baxter_interface.Limb('left')
        gripper = baxter_interface.Gripper('left')
        gripper.calibrate()
        pub = rospy.Publisher("eeg_output", Int32)
        rospy.Subscriber("command", Int32, run_experiment, callback_args=(arduino_pub, baxter_limb, gripper, pub))
        
        print "started main node"
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
