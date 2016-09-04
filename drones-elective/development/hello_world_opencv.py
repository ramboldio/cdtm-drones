# # # -*- coding: utf-8 -*-

import cv2
import libardrone.libardrone as libardrone

print cv2.__version__

drone = libardrone.ARDrone()

cam = cv2.VideoCapture(0)  # Get the stream of your webcam
# cam = cv2.VideoCapture('tcp://192.168.1.1:5555')

running = True

while running:
    running, frame = cam.read()

    if running:
        # cv2.imshow('frame', frame)
        cv2.imshow("Drone camera", cv2.cvtColor(drone.get_image(), cv2.COLOR_BGR2RGB))

    if cv2.waitKey(1) & 0xFF == 27:
        running = False
    else:
        print 'error reading video feed'

cam.release()

cv2.destroyAllWindows()
#
# import cv2
#
# cam = cv2.VideoCapture('tcp://192.168.1.1:5555')
#
# running = True
#
# while running:
#     # get current frame of video
#     running, frame = cam.read()
#
#     if running:
#         cv2.imshow('frame', frame)
#         if cv2.waitKey(1) & 0xFF == 27:
#             # escape key pressed
#             running = False
#     else:
#         # error reading frame
#         print 'error reading video feed'
#
# cam.release()
#
# cv2.destroyAllWindows()
