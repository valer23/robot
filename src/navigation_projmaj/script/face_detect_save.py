#!/usr/bin/env python

import face_detect
import time


def main(counter):
    fd = face_detect.FaceDetect()
    fd.num = counter
    fd.face_detection()

    return True


if __name__ == '__main__':
    count = 0
    r = True
    while r:
        r = main(count)
        count += 1
        time.sleep(3)
        print('Number of pictures saved:', count)
