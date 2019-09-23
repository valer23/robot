#!/usr/bin/env python

import cv2


class FaceDetectSimple:

    def __init__(self):
        # Get user supplied values
        self.imagePath = "test.jpg"
        cascPath = "haarcascade_frontalface_default.xml"
        # cascPath = "cascade_camion3_LBP.xml"

        # Create the haar cascade
        self.faceCascade = cv2.CascadeClassifier(cascPath)

        # Read the image
        imageFullSize = cv2.imread(self.imagePath)
        self.image = cv2.resize(imageFullSize, (972, 648))
        # image = cv2.imread(imagePath)

    def detection(self):
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Detect faces in the image
        faces = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.4,
            minNeighbors=7,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        print("Found {0} faces!".format(len(faces)))

        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            cv2.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # cv2.imshow("Faces found", image)
        self.display_img(self.image)
        # new_name = "./detection/detect_" + self.imagePath
        # cv2.imwrite(new_name, self.image)
        # cv2.waitKey(0)

    @staticmethod
    def display_img(img):
        cv2.imshow('Test', img)
        cv2.waitKey(0)


if __name__ == '__main__':
    f = FaceDetectSimple()
    f.detection()
