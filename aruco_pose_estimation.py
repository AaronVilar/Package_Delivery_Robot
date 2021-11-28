import numpy as np
from utils import ARUCO_DICT, aruco_display
import argparse
import time
import cv2
import sys
import math

video = cv2.VideoCapture(1, cv2.CAP_DSHOW)
time.sleep(2.0)

matrix_coefficients = np.load(r"C:\Users\aaron\PycharmProjects\aruco_markers\calibration_matrix.npy")
distortion_coefficients = np.load(r"C:\Users\aaron\PycharmProjects\aruco_markers\distortion_coefficients.npy")

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
arucoParams = cv2.aruco.DetectorParameters_create()

r = np.zeros((3, 1))
t = np.zeros((3, 1))

R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

font = cv2.FONT_HERSHEY_PLAIN


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


while True:

    ret, frame = video.read()
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    arucoParams = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, arucoDict,
                                                                parameters=arucoParams,
                                                                cameraMatrix=matrix_coefficients,
                                                                distCoeff=distortion_coefficients)
    if len(corners) > 0:
        for i in range(0, len(ids)):
            ret = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

            cv2.aruco.drawDetectedMarkers(frame, corners)

            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

            # -- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                math.degrees(roll_marker), math.degrees(pitch_marker),
                math.degrees(yaw_marker))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # -- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc * np.matrix(tvec).T

            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (pos_camera[0], pos_camera[1], pos_camera[2])
            cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # -- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                math.degrees(roll_camera), math.degrees(pitch_camera),
                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("Estimated Pose", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
video.release()
