#!/usr/bin/env python

import face_detect
import face_crop
import time


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

    return True


if __name__ == '__main__':
    count = 110
    r = True
    while r:
        r = main(count)
        count += 1
        time.sleep(3)
        print('Number of pictures saved:', count)
