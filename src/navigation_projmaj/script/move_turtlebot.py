#!/usr/bin/env python

import rospy
import actionlib
import dlib_reco_image
# import face_detect
#import servomotor
import play_sound
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from kobuki_msgs.msg import ButtonEvent


class MapNavigation:

    def __init__(self):

        self.button = None
        # self.fd = face_detect.FaceDetect()

        # declare the coordinates of interest
        self.xOffice1 = -16.2940
        self.yOffice1 = -15.6076
        self.xOffice2 = -5.602
        self.yOffice2 = -2.5
        self.xOffice3 = -27.946
        self.yOffice3 = -30.053
        self.goalReached = False

        # initiliaze
        rospy.init_node('map_navigation', anonymous=False)
        # monitor kobuki's button events
        rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.button_event_callback)

    def move_after_pressed(self):
        if self.button == 1:
            self.goalReached = self.move_to_goal(self.xOffice1, self.yOffice1)
        elif self.button == 2:
            self.goalReached = self.move_to_goal(self.xOffice2, self.yOffice2)
        elif self.button == 3:
            self.goalReached = self.move_to_goal(self.xOffice3, self.yOffice3)

        # if self.goalReached:
        #     s = servomotor.Servomotor()
        #     print('Allowed')
        #     s.mode = True
        # self.launch_face_detect(self.goalReached)

    def button_event_callback(self, data):
        if data.state == ButtonEvent.RELEASED :
            state = "released"
        else:
            state = "pressed"
        if data.button == ButtonEvent.Button0:
            self.button = 1
        elif data.button == ButtonEvent.Button1:
            self.button = 2
        else:
            self.button = 3

        self.move_after_pressed()
        rospy.loginfo("Button %s was %s." % (self.button, state))

    # def launch_face_detect(self, goal):
    #     if goal:
    #         self.fd.face_detection()
    #         if self.fd.saved:
    #             ri = dlib_reco_image.FaceRecognition()
    #             allowed = ri.who_is_there('detected/detected_person.jpg')
    #             if allowed:
    #                 # play_sound.play_music('music/zelda.mp3')
    #                 s = servomotor.Servomotor()
    #                 s.mode = True
    #             else:
    #                 print('You are not allowed')
    #         else:
    #             print('No image saved')
    #     else:
    #         print('Goal not reached')

    @staticmethod
    def shutdown():

        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep(20)

    def move_to_goal(self, xGoal, yGoal):
        # define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for the action server to come up
        while not ac.wait_for_server(rospy.Duration.from_sec(5.0)):
            rospy.loginfo("Waiting for the move_base action server to come up")
        goal = MoveBaseGoal()
        # play_sound.play_music('music/pokemon.mp3')

        # set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/
        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result()

        #if ac.get_result() is not None:
        #    rospy.loginfo("You have reached the destination")
        #    return True

        #else:
        #    rospy.loginfo("The robot failed to reach the destination")
        #    return False


if __name__ == '__main__':
    try:
        rospy.loginfo("You have reached the destination")
        m = MapNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
