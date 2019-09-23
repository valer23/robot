#!/usr/bin/env python

import face_recognition


class FaceRecognition:

    val_encoding = None
    flo_encoding = None

    def __init__(self):
        val = face_recognition.load_image_file('known_person/val.jpg')
        flo = face_recognition.load_image_file('known_person/flo.jpg')

        self.val_encoding = face_recognition.face_encodings(val)[0]
        self.flo_encoding = face_recognition.face_encodings(flo)[0]

    def who_is_there(self, img_path):
        unknown_image = face_recognition.load_image_file()
        unknown_encoding = face_recognition.face_encodings(unknown_image)[0]

        if face_recognition.compare_faces([self.val_encoding], unknown_encoding) == [True]:
            print('Val is in the picture.')
            return True
        elif face_recognition.compare_faces([self.flo_encoding], unknown_encoding) == [True]:
            print('Flo is in the picture.')
            return True
        else:
            print('I do not know who is in the picture.')
            return False


if __name__ == '__main__':
    fr = FaceRecognition()
    path = 'detected/detected_person.jpg'
    fr.who_is_there(path)




# image = face_recognition.load_image_file('test.jpg')
# face_locations = face_recognition.face_locations(image)
# # print(face_locations)
# face_landmarks_list = face_recognition.face_landmarks(image)
#
# known_image = face_recognition.load_image_file('known_person/val.jpg')
# unknown_image = face_recognition.load_image_file('unknown/unknown1.jpg')
#
# known_encoding = face_recognition.face_encodings(known_image)[0]
# print(biden_encoding)
# unknown_encoding = face_recognition.face_encodings(unknown_image)[0]
#
# results = face_recognition.compare_faces([known_encoding], unknown_encoding)
#
# print(results)
# if results == [True]:
#     print('OK')
