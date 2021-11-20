import cv2 as cv2
import numpy as np
import imutils
from smbus import SMBus
import time

addr = 0x8
wheel_speed_cmd = 10
bus = SMBus(1)
time.sleep(5)


def nothing(x):
    pass


# Load image
cap = cv2.VideoCapture(0)

# Create a window
cv2.namedWindow('image')

# Initialize HSV min/max values
hMin = 41
sMin = 50
vMin = 10
hMax = 90
sMax = 255
vMax = 255

while (1):

    ret, image = cap.read()

    # Set minimum and maximum HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Convert to HSV format and color threshold
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(image, image, mask=mask)

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
        print(center)

        input_send = 1000
        if center[0]-320 < 0:
            sign_bit_R, sign_bit_L = 1, 0
            input_send = input_send * -1
        else:
            sign_bit_R, sign_bit_L = 1, 0

        binary = bin(input_send)
        binary_len = len(binary)
        if binary_len <= 10:
            print(wheel_speed_cmd, 4, [sign_bit_L, sign_bit_R, input_send, input_send])
            bus.write_block_data(addr, wheel_speed_cmd, [sign_bit_L, sign_bit_R, input_send, input_send])
        else:
            byte_1 = int(binary[2:binary_len - 8], 2)
            byte_2 = int(binary[binary_len - 8:binary_len], 2)
            print(wheel_speed_cmd, 2, [sign_bit_L, byte_1, byte_2, sign_bit_R, byte_1, byte_2])
            bus.write_block_data(addr, wheel_speed_cmd, [sign_bit_L, sign_bit_R, byte_1, byte_2, byte_1, byte_2])
            # bus.write_word_data(addr, byte_1, byte_2)

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.imshow('image',cv2.circle(result, (int(x), int(y)), int(radius), (0, 255, 255), 2))

            cv2.imshow('image', cv2.circle(result, center, 5, (0, 0, 255), -1))
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

cv2.destroyAllWindows()
