#!/usr/bin/env python

import cv2
import time


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
    num = None

    def __init__(self):
        # Get user supplied values
        self.cap = cv2.VideoCapture(1)
        self.image_name = 'person.jpg'
        self.cascade_path = 'haarcascade_frontalface_alt.xml'
        ret, frame = self.cap.read()

        # Create the haar cascade
        self.face_cascade = cv2.CascadeClassifier(self.cascade_path)

        # # Read the image ad resize it if needed
        # # image_full_size = cv2.imread(self.image_name)
        image_full_size = frame
        # height, width = image_full_size.shape[:2]
        # if width > 1000 or height > 1000:
        #     self.image = cv2.resize(image_full_size, (int(width/2), int(height/2)))
        # else:
        self.image = image_full_size

        # Transform the RGB image to GRAY image
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def display_image(img):
        cv2.imshow('Faces found', img)
        cv2.waitKey(0)

    def save_image(self):
        number = self.num
        if number is not None:
            new_name = 'detected/detected_' + str(number) + self.image_name
        else:
            new_name = 'detected/detected_' + self.image_name

        cv2.imwrite(new_name, self.image)
        cv2.waitKey(10)
        print('SAVED')
        self.saved = True

    def shut_down_cam(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def draw_rectangle(self, x, y, w, h):
        cv2.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 1)

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

        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            self.x_detect.append(x)
            self.y_detect.append(y)
            self.w_detect.append(w)
            self.h_detect.append(h)
            # self.draw_rectangle(x, y, w, h)

        # self.display_image(self.image)
        self.save_image()
        cv2.waitKey(100)

        return faces

