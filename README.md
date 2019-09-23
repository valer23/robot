# Projet Majeur at CPE Lyon
5th year of Robotics Sciences
Group 7A

## Description of the project
Robot based on the Turtlebot mobile base.
This robot is going to be used as a "postman robot" with a special highlighting
on the confidentiality.

We implemented the Robot as an autonomous entity, he is able to move by it-self
in a previous known area. So it can deliver its mails to the right receiver.
For the confidential part, we are going to use face recognition base on deep
learning and a camera on the top of the robot.

The robot is going to take the mails into a locked box and it is going to open
this box only in front of the right person, after a phase of face regnition.

### Material description
- Turtlebot mobile base
- RGB camera
- at least one laptop/computer
- a mail box


## I. SIMULATION MAPPING WITH RVIZ & STAGE

What we do here:
- usage of training-turtlebot-simulator-student folder used previously during ROBOTIQUE MOBILE course
- modification of turtlebot_stage files

### 1) In the "maps" folder : files to visualize the 3rd floor during the mapping

- insertion of the map Etage3_yaml.pgm
- insertion of the file Etage3_14.yaml


### 2) In the "maps/stage" folder

- creation of "etage3.world" file (necessary for the good launchfile's functioning)
- management of parameters to size properly the map (floorplan) et initialize the position of the Turtlebot with Stage


  ```xml
floorplan
(
	  name "Etage3_14"
	  bitmap "../Etage3_14.pgm"
	  size [ 100.0 100.0 2.0 ]
	  pose [  5.0  5.0 0.0 0.0 ]
)
turtlebot
(
  pose [ 40.0 15.0 0.0 0.0 ]
  name "turtlebot"
  color "black"
)
  ```

### 3) In the folder launch : to run the map of the 3rd floor

- modification of the turtlebot_in_stage.launch file
- modification of the path in the map and world file
- management of parameters to initialize properly the Turtlebot with Rviz


```xml
<arg name="map_file"       default=" $(find turtlebot_stage)/maps/Etage3_14.yaml"/>
<arg name="world_file"     default=" $(find turtlebot_stage)/maps/stage/etage3.world"/>
<arg name="initial_pose_x" default="-6.0"/>
<arg name="initial_pose_y" default="-2.0"/>
<arg name="initial_pose_a" default="0.0"/>
```

Note :
- it is important to position properly the Turtlebot at the same position with Rviz and Stage

Comments :
- waste of time during the positioning because the map with Rviz is misdirected
- good simulaton for a simple movement
- difficulties to set the same orientation with Rviz and Stage
- cumulative error during the movement

### 4) Simulation of the movement for three precise positions with Rviz

What we do here :
- creation of map_navigation.py
- the file will ask which position we want to choose (office1 , office2 or office3) in the 3rd floor.
- the user writes 1 for Office1, 2 for Office2 and 3 for Office 3.

```python
#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from math import radians, degrees
#from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

class map_navigation():
    def choose(self):

	self.choice = 4

	rospy.loginfo("|-------------------------------|")
	rospy.loginfo("|PRESS A KEY:")
	rospy.loginfo("|'1': Office 1 ")
	rospy.loginfo("|'2': Office 2 ")
	rospy.loginfo("|'3': Office 3 ")
	rospy.loginfo("|-------------------------------|")
	rospy.loginfo("|WHERE TO GO? CHOOSE AND PRESS ENTRY")
	choice = input()
	return choice

    def __init__(self):

	# declare the coordinates of interest
	self.xOffice1 = -20.139
	self.yOffice1 = -20.378
	self.xOffice2 = -5.2911
	self.yOffice2 = -2.858
	self.xOffice3 = -19.799
	self.yOffice3 = 8.581
	self.goalReached = False

	# initiliaze
	rospy.init_node('map_navigation', anonymous=False)
	choice = self.choose()
	if (choice == 1):
	    self.goalReached = self.moveToGoal(self.xOffice1, self.yOffice1)
	elif (choice == 2):
	    self.goalReached = self.moveToGoal(self.xOffice2, self.yOffice2)
	elif (choice == 3):
	    self.goalReached = self.moveToGoal(self.xOffice3, self.yOffice3)

	if (choice != 4):
	    if (self.goalReached):
	        rospy.loginfo("Congratulations!")
	        # rospy.spin()
	    # rospy.spin()
	    else:
	        rospy.loginfo("Hard Luck!")
	else:
	    self.shutdown()

	while choice != 4:
	    choice = self.choose()
	    if (choice == 1):
	        self.goalReached = self.moveToGoal(self.xOffice1, self.yOffice1)
	    elif (choice == 2):
	        self.goalReached = self.moveToGoal(self.xOffice2, self.yOffice2)
	    elif (choice == 3):
	        self.goalReached = self.moveToGoal(self.xOffice3, self.yOffice3)

	    if (choice != 4):
	        if (self.goalReached):
	            rospy.loginfo("Congratulations!")
	            # rospy.spin()
	        else:
	            rospy.loginfo("Hard Luck!")

    def shutdown(self):

	# stop turtlebot
	rospy.loginfo("Quit program")
	rospy.sleep(20)

    def moveToGoal(self, xGoal, yGoal):

	# define a client for to send goal requests to the move_base server through a SimpleActionClient
	ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	# wait for the action server to come up
	while (not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
	    rospy.loginfo("Waiting for the move_base action server to come up")
	goal = MoveBaseGoal()

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

	if (ac.get_result() != None):
	    rospy.loginfo("You have reached the destination")
	    return True

	else:
	    rospy.loginfo("The robot failed to reach the destination")
	    return False

if __name__ == '__main__':
    try:
	rospy.loginfo("You have reached the destination")
	map_navigation()
	rospy.spin()
    except rospy.ROSInterruptException:
```

Comments :
- precise movement of the turtlebot
- good simulation, we can test in a real situation



## II. REAL MAPPING REEL AU 3ETAGE AVEC LE ROBOT TURTLEBOT

### 1) Connection between the turtlebot laptop and the computer

What we do here:
- usage of Turtlebot training
- connection between the computer and the turtlebot laptop



Comments :
- difficulties to connect the computer and the laptop because of the Wifi connection
- many update/upgrade to do

### 2) Real movement

What we do here :

- creation of move_turtlebot.py
- on the turtlebot, the user push on the Button0 (B0) for Office1, Button1 (B1) for Office2 and Button2 (B2) for Office 3.

```python
#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from kobuki_msgs.msg import ButtonEvent


class MapNavigation():

def __init__(self):

  self.button = None

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
  if (self.button == 1):
      self.goalReached = self.move_to_goal(self.xOffice1, self.yOffice1)
  elif (self.button == 2):
      self.goalReached = self.move_to_goal(self.xOffice2, self.yOffice2)
  elif (self.button == 3):
      self.goalReached = self.move_to_goal(self.xOffice3, self.yOffice3)


def button_event_callback(self, data):
if (data.state == ButtonEvent.RELEASED):
      state = "released"
else:
    state = "pressed"
if (data.button == ButtonEvent.Button0):
    self.button = 1
elif (data.button == ButtonEvent.Button1):
    self.button = 2
else:
    self.button = 3

  self.move_after_pressed()
  rospy.loginfo("Button %s was %s." % (self.button, state))


def shutdown(self):

  # stop turtlebot
  rospy.loginfo("Quit program")
  rospy.sleep(20)

def move_to_goal(self, xGoal, yGoal):

  # define a client for to send goal requests to the move_base server through a SimpleActionClient
  ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

  # wait for the action server to come up
  while (not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
      rospy.loginfo("Waiting for the move_base action server to come up")
  goal = MoveBaseGoal()

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

  #if (ac.get_result() != None):
     # rospy.loginfo("You have reached the destination")
     #return True

  #else:
      #rospy.loginfo("The robot failed to reach the destination")
      #return False


if __name__ == '__main__':
try:
  rospy.loginfo("You have reached the destination")
  m = MapNavigation()
  rospy.spin()
except rospy.ROSInterruptException:
  rospy.loginfo("map_navigation node terminated.")
```

Comments :
- precise movement of the turtlebot
- good simulation, we can test in a real situation

## III. Opening/Closing the mail box with one servomotor Dynamixel AX-12


We use one Dynamixel servomotor AX-12 to open/close the mailbox.

What we do here :
- usage of http://wiki.ros.org/dynamixel_controllers/Tutorials to configurate the Dynamixel servomotor
- creation of a package my-dynamixelNAME with dynamixel_controllers std-msgs rospy roscpp
- connection to /dev/tty/ACM0 port

### 1) In the file launch/controller_manager.launch

- the controller manager will connect to the motor and publish raw feedback (current_position, goal position etc)
- creation of the file controller_manager.launch
- modification of the port name

```xml
<launch>
    <node name="dynamixel_manager" pkg="dynamixel_controllers" type="controller_manager.py" required="true" output="screen">
	<rosparam>
	    namespace: dxl_manager
	    serial_ports:
	        pan_tilt_port:
	            port_name: "/dev/ttyACM0"
	            baud_rate: 1000000
	            min_motor_id: 1
	            max_motor_id: 25
	            update_rate: 20
	</rosparam>
    </node>
</launch>
```

### 2) In the file tilt.yaml

- this file is a configuration file that will contain all parameters that are necessary for our controller
- check of the id (rostopic echo /motor_states/pan_tilt_port)
- creation of the file tilt.yaml
- modification of the id

```xml
joint1_controller:
    controller:
        package: dynamixel_controllers
        module: joint_position_controller
        type: JointPositionController
    joint_name: tilt_joint
    joint_speed: 1.0
    motor:
        id: 1
        init: 512
        min: 0
        max: 1023
```

### 3) In the file launch/start_tilt_controller.launch

- this file will load controller parameters to the parameter server and start up the controller
- modification of path and the name of the controller (here joint1_controller)

```xml
<launch>
    <!-- Start tilt joint controller -->
    <rosparam file="$(find my_dynamixel_tutorial)/launch/tilt.yaml" command="load"/>
    <node name="tilt_controller_spawner" pkg="dynamixel_controllers" type="controller_spawner.py"
	  args="--manager=dxl_manager
	        --port pan_tilt_port
	        joint1_controller"
	  output="screen"/>
</launch>
```

### 4) In the file script/servomotor.py

- this file will control the mail box. It receives a boolean and the box is initially closed
- if the file receives "True", the box will be opened during 15s

```python
#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState


class Servomotor:
    # if mode = False, the box is closed
    # else, open

    mode = False
    angle_motor_1 = 0.0

    def __init__(self):
        rospy.init_node('servomotor', anonymous=True)
        self.pub = rospy.Publisher('/joint1_controller/command', Float64, queue_size=10)

    def open_box(self):
        self.angle_motor_1 = 1.5
        rospy.loginfo("Open Box")
        self.pub.publish(self.angle_motor_1)

    def close_box(self):
        self.angle_motor_1 = 0.0
        rospy.loginfo("Close Box")
        self.pub.publish(self.angle_motor_1)

    def callback(self):

        if self.mode == False:
            self.close_box()
            time.sleep(2)
        if self.mode == True:
            self.open_box()
            time.sleep(15)
            self.mode = False

        rospy.loginfo(self.angle_motor_1)

    def listener(self):
        rospy.Subscriber('/joint1_controller/state', JointState, self.callback)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        s = Servomotor()
        s.listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

```


## IV. Face detection and recognition

In order to unload the amount of computing stuff on the Turtlebot computer, we will have to work on small pictures for the face recognition.
Here we are going to detect if someone is in front of the turtlebot when it arrives at the good location. If someone is standing up in front of the camera, we use OpenCV library with Haar Cascade method to detect the face of the person. Then we have to define the size of the cropping, to do that we use the parameters of the rectangle boxes drown by OpenCV. We use those parameters for PIL.Image librairy and crop the picture around the face(s) detected in the video stream (camera).
Finally we send those cropped faces to the command computer (more powerful) to run the CNN facial recognition on those pictures.
(See python script into python scripts folder)

### 1) face_detect.py

```python
#!/usr/bin/env python

import cv2


class FaceDetect:

    x_detect = []
    y_detect = []
    h_detect = []
    w_detect = []
    image_name = None
    cascade_path = None
    face_cascade = None
    image = None
    gray = None
    saved = False
    cap = None

    def __init__(self):
        # Get user supplied values
        self.cap = cv2.VideoCapture(0)
        self.image_name = 'imageVal.jpg'
        self.cascade_path = 'haarcascade_frontalface_alt.xml'
        ret, frame = self.cap.read()

        # Create the haar cascade
        self.face_cascade = cv2.CascadeClassifier(self.cascade_path)

        # Read the image ad resize it if needed
        # image_full_size = cv2.imread(self.image_name)
        image_full_size = frame
        height, width, channels = image_full_size.shape
        if width > 1000 or height > 1000:
            self.image = cv2.resize(image_full_size, (int(width/2), int(height/2)))
        else:
            self.image = image_full_size

        # Transform the RGB image to GRAY image
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def display_image(img):
        cv2.imshow('Faces found', img)
        cv2.waitKey(0)

    def save_image(self):
        new_name = 'detected/detected_' + self.image_name
        cv2.imwrite(new_name, self.image)
        cv2.waitKey(10)
        print('SAVED')
        self.saved = True

    def face_detection(self):
        # Detect faces in the image
        faces = self.face_cascade.detectMultiScale(
            self.gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(50, 50),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        print('Found {0} faces!'.format(len(faces)))

        print(faces)
        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            self.x_detect.append(x)
            self.y_detect.append(y)
            self.w_detect.append(w)
            self.h_detect.append(h)
            cv2.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # self.display_image(self.image)
        self.save_image()
        cv2.waitKey(5000)
        self.cap.release()
        cv2.destroyAllWindows()

        return faces
```

### 2) face_crop.py

```python
#!usr/bin/env python

from PIL import Image


class FaceCrop:

    image_path = None
    name_image = None
    cpt = 0

    def __init__(self):
        self.name_image = 'imageVal.jpg'
        self.image_path = 'detected/detected_'

    @staticmethod
    def display_area(area):
        area.show()

    @staticmethod
    def save_area(name_img, number, area):
        new_name = 'cropped/cropped_' + str(number) + '_' + name_img
        area.save(new_name)

    def crop(self, x, y, w, h):
        # Download Image:
        im = Image.open(self.image_path + self.name_image)

        # Check Image Size
        im_size = im.size
        # print(im_size)

        for i in range(len(x)):
            # Define box inside image
            left = x[i]
            top = y[i]
            width = w[i]
            height = h[i]

            # Create Box
            box = (left, top, left + width, top + height)

            # Crop Image
            area = im.crop(box)
            # self.display_area(area)
            # area.show()

            # Save Image
            # print(area.size)
            self.save_area(self.name_image, self.cpt, area)
```

### 3) face_detect_crop.py

```python
#!/usr/bin/env python

import face_detect
import face_crop


def main(counter):
    fd = face_detect.FaceDetect()
    fd.face_detection()

    x = fd.x_detect
    y = fd.y_detect
    w = fd.w_detect
    h = fd.h_detect

    if fd.saved:
        fc = face_crop.FaceCrop()
        fc.cpt = counter
        fc.crop(x, y, w, h)
        print('Detection & Cropping done!')
    else:
        print('Detection had not been saved.')


if __name__ == '__main__':
    count = 0
    while True:
        main(count)
        count += 1
```

### CNN training

We use two Github tutorial for the Convolutional Neural Network (CNN):

TensorFlow Tutorial - https://github.com/EdjeElectronics/TensorFlow-Object-Detection-API-Tutorial-Train-Multiple-Objects-Windows-10

LabelImg Tutorial - https://github.com/tzutalin/labelImg


Unfortunately the CNN do not work probably because of a data set problem.
That is why we will use the python DLIB library for the face recognition.

### DLIB face recognition
Here we have the script code for the face recognition using DLIB.

```python
#!/usr/bin/env python

import face_recognition
import cv2

video_capture = cv2.VideoCapture(0)

# Load a sample picture and learn how to recognize it.
val_image = face_recognition.load_image_file("known_person/val.jpg")
val_face_encoding = face_recognition.face_encodings(val_image)[0]

# Load a second sample picture and learn how to recognize it.
flo_image = face_recognition.load_image_file("known_person/flo.jpg")
flo_face_encoding = face_recognition.face_encodings(flo_image)[0]

# Create arrays of known face encodings and their names
known_face_encodings = [
    val_face_encoding,
    flo_face_encoding
]
known_face_names = [
    "Valerian",
    "Florie"
]

# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True

while True:
    # Grab a single frame of video
    ret, frame = video_capture.read()

    # Resize frame of video to 1/4 size for faster face recognition processing
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_small_frame = small_frame[:, :, ::-1]

    # Only process every other frame of video to save time
    if process_this_frame:
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            name = "UNKNOWN"

            # If a match was found in known_face_encodings, just use the first one.
            if True in matches:
                first_match_index = matches.index(True)
                name = known_face_names[first_match_index]

            face_names.append(name)

    process_this_frame = not process_this_frame


    # Display the results
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4

        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 1)

        # Draw a label with a name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 255, 0), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

    # Display the resulting image
    cv2.imshow('Video', frame)

    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()

```

This library comes all set, working on a CNN already trained for facial recognition. What we need to do is to give it the faces we want to recognize.
Those faces pictures are located into "known_person" folder.


### V. Configuration and using
#### Dependencies

To use those scripts you will need to install some python libraries on both computer.
- opencv (at least v3.0.0)
- dlib
- pygame
- face_recognition
- numpy

[

I would advice you to install all of those libraries into a python virtual environnement.
- virtualenv
- virtualenvwrapper

]

#### Configuration of the communication
- Connect both computer (remote & turtlebot) at the same Wifi network
- First you have to change the .bashrc file to communicate between the remote PC and the turtlebot
- Follow the instructions given into "init_config.txt" file in the git folder.


#### Launch navigation
- When the connection is established launch the navigation python script "move_turtlebot.py"
- Launch RVIZ on the turtlebot
- Choose your goal location (office) by pressing one of the buttons located on the turtlebot.

#### Face recognition
As the face recognition does not work automatically due to a lack of memory size on the turtlebot. We have to launch the "dlib_reco_video.py" in an other terminal.
- Launch dlib_reco_video.py
If the person recognized is allowed to receive the mail then the box should open. After 15 seconds the box closes.


### VI. Videos
[Link to the pitch YouTube video](https://www.youtube.com/watch?v=Ch_LK5oZdqk)

[Link to the tutorial YouTube video](https://www.youtube.com/watch?v=4iGXVcpAS6g)


Valérian BART - Florie CHABRA