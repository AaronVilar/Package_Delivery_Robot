
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2 as cv2
import imutils
import time

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (100, 150, 0)
greenUpper = (140, 255, 255)

# if a video path was not supplied, grab the reference
# to the webcam
vs = cv2.VideoCapture(1)


# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
while True:
	# grab the current frame
	ret, frame = vs.read()

	# resize the frame, blur it, and convert it to the HSV
	# color space
	# frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	# mask_display = np.expand_dims(mask, 2)
	# mask_display = np.concatenate((mask_display, mask_display, mask_display), axis=2)
	# top_display = np.concatenate((frame, mask_display), axis=1)

	mask = cv2.erode(mask, None, iterations=2)
	# mask_display = np.expand_dims(mask, 2)
	# bottom_display = np.concatenate((mask_display, mask_display, mask_display), axis=2)

	mask = cv2.dilate(mask, None, iterations=2)
	# mask_display = np.expand_dims(mask, 2)
	# mask_display = np.concatenate((mask_display, mask_display, mask_display), axis=2)
	# bottom_display = np.concatenate((bottom_display, mask_display), axis=1)
	# display = np.concatenate((top_display, bottom_display), axis=0)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

			cv2.circle(frame, center, 5, (0, 0, 255), -1)

	# show the frame to our screen
	# cv2.imshow("Frame", display)
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

vs.stop()

# otherwise, release the camera
vs.release()

# close all windows
cv2.destroyAllWindows()
