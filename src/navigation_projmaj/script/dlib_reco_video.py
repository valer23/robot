#!/usr/bin/env python

import face_recognition
import cv2


#
# class RecognitionVideo:
#
#     # Initialize some variables
#     face_locations = []
#     face_encodings = []
#     face_names = []
#     process_this_frame = True
#     # Get the video stream from webcam
#     video_capture = cv2.VideoCapture(1)
#     # Grab a single frame of video
#     ret, frame = video_capture.read()
#
#     def __init__(self):
#         # # Get the video stream from webcam
#         # self.video_capture = cv2.VideoCapture(1)
#         # # Grab a single frame of video
#         # self.ret, self.frame = self.video_capture.read()
#
#         # Load the face picture and learn how to recognize it.
#         val_image = face_recognition.load_image_file("known_person/val.jpg")
#         val_face_encoding = face_recognition.face_encodings(val_image)[0]
#         flo_image = face_recognition.load_image_file("known_person/flo.jpg")
#         flo_face_encoding = face_recognition.face_encodings(flo_image)[0]
#
#         # Create arrays of known face encodings and their names
#         self.known_face_encodings = [
#             val_face_encoding,
#             flo_face_encoding
#         ]
#         self.known_face_names = [
#             "Valerian",
#             "Florie"
#         ]
#
#     def do_i_know_you(self):
#         # Resize frame of video to 1/4 size for faster face recognition processing
#         small_frame = cv2.resize(self.frame, (0, 0), fx=0.25, fy=0.25)
#
#         # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
#         rgb_small_frame = small_frame[:, :, ::-1]
#
#         # Only process every other frame of video to save time
#         if self.process_this_frame:
#             # Find all the faces and face encodings in the current frame of video
#             self.face_locations = face_recognition.face_locations(rgb_small_frame)
#             self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations)
#
#             self.face_names = []
#             for face_encoding in self.face_encodings:
#                 # See if the face is a match for the known face(s)
#                 matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
#                 name = "UNKNOWN"
#
#                 # If a match was found in known_face_encodings, just use the first one.
#                 if True in matches:
#                     first_match_index = matches.index(True)
#                     name = self.known_face_names[first_match_index]
#
#                 self.face_names.append(name)
#
#         self.process_this_frame = not self.process_this_frame
#
#         self.display_results()
#
#     def display_results(self):
#         # Display the results
#         for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
#             # Scale back up face locations since the frame we detected in was scaled to 1/4 size
#             top *= 4
#             right *= 4
#             bottom *= 4
#             left *= 4
#
#             # Draw a box around the face
#             cv2.rectangle(self.frame, (left, top), (right, bottom), (255, 0, 0), 1)
#
#             # Draw a label with a name below the face
#             cv2.rectangle(self.frame, (left, bottom - 35), (right, bottom), (0, 255, 0), cv2.FILLED)
#             font = cv2.FONT_HERSHEY_DUPLEX
#             cv2.putText(self.frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
#
#         # Display the resulting image
#         cv2.imshow('Video', self.frame)
#
#
# if __name__ == '__main__':
#     rv = RecognitionVideo()
#     while True:
#         rv.do_i_know_you()
#         # Hit 'q' on the keyboard to quit!
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     print(rv.face_names)
#     # Release handle to the webcam
#     rv.video_capture.release()
#     cv2.destroyAllWindows()



# Get a reference to webcam #0 (the default one)
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
