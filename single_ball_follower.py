# poprawić. Na początku wyizolować obrac, wykryć kulki a później spawdzać ich kolor
# https://www.youtube.com/watch?v=aHTVDoOWYB8
import time

import imutils
from imutils.video import VideoStream
import serial
import numpy as np
import cv2


def translate(value, oldMin, oldMax, newMin=-100, newMax=100):
    # Figure out how 'wide' each range is
    oldRange = oldMax - oldMin
    newRange = newMax - newMin
    NewValue = (((value - oldMin) * newRange) / oldRange) + newMin
    return int(NewValue)


usesPiCamera = False

cameraResolution = (640, 480)

vs = VideoStream(usePiCamera=usesPiCamera, resolution=cameraResolution, framerate=60).start()
time.sleep(2.0)

colorLower = (0, 0, 0)
colorUpper = (255, 255, 255)
colorTolerance = 3
paused = False
roiSize = (16, 16)  # roi size on the scaled down image (converted to HSV)

def nothing(x):
    pass

cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 0, 255, nothing) #low hue
cv2.createTrackbar("LS", "Tracking", 0, 255, nothing) #low saturation
cv2.createTrackbar("LV", "Tracking", 0, 255, nothing) #low value
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing) #upper hue
cv2.createTrackbar("US", "Tracking", 255, 255, nothing) #upper saturation
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing) #upper value


# # initialize serial communication
# ser = serial.Serial(port='/dev/ttyACM0', baudrate=57600, timeout=0.05)

while True:
    # for cameraFrame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    loopStart = time.time()
    if not paused:

        frame = vs.read()
        # frame = cv2.flip(frame, flipCode=-1)
        time.sleep(0.1)
        height, width = frame.shape[0:2]
        scaleFactor = 1
        newWidth, newHeight = width // scaleFactor, height // scaleFactor

        resizedColor = cv2.resize(frame, (newWidth, newHeight), interpolation=cv2.INTER_CUBIC)
        resizedColor_blurred = cv2.GaussianBlur(resizedColor, (11, 11), 0)

        # resizedHSV = cv2.cvtColor(resizedColor, cv2.COLOR_BGR2HSV)
        resizedHSV = cv2.cvtColor(resizedColor_blurred, cv2.COLOR_BGR2HSV)

        # roi = resizedHSV[newHeight // 2 - roiSize[0] // 2: newHeight // 2 + roiSize[0] // 2,
        #       newWidth // 2 - roiSize[1] // 2: newWidth // 2 + roiSize[1] // 2, :]

        # roi = resizedHSV

        l_h = cv2.getTrackbarPos("LH", "Tracking")
        l_s = cv2.getTrackbarPos("LS", "Tracking")
        l_v = cv2.getTrackbarPos("LV", "Tracking")

        u_h = cv2.getTrackbarPos("UH", "Tracking")
        u_s = cv2.getTrackbarPos("US", "Tracking")
        u_v = cv2.getTrackbarPos("UV", "Tracking")


        colorLowerWithTolerance = np.array([l_h, l_s, l_v])  # low blue - set low value of colour
        colorUpperWithTolerance = np.array([u_h, u_s, u_v])  # upper blue - set upper value of colour

        mask = cv2.inRange(resizedHSV, colorLowerWithTolerance, colorUpperWithTolerance)
        # mask = cv2.inRange(resizedHSV, l_h, u_h)


        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        cv2.erode(mask, None, iterations=2)
        cv2.dilate(mask, None, iterations=2)

        # (_, contours, hierarchy) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        # contours = imutils.grab_contours(contours)
        center = None
        biggestObjectMiddle = None

        upscaledColor = cv2.resize(resizedColor, (width, height), interpolation=cv2.INTER_NEAREST)

        # only proceed if at least one contour was found
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(contours, key=cv2.contourArea)
            ((X, Y), Radius) = cv2.minEnclosingCircle(c)
            x = int(X)
            y = int(Y)
            radius = int(Radius)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 3, (0, 0, 255), -1)
                cv2.putText(frame, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),
                            1)
                cv2.putText(frame, "(" + str(center[0]) + "," + str(center[1]) + ")", (center[0] + 10, center[1] + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                # print("Object : ({},{}); radius: {};".format(x, y, radius))
                screenMiddle = width // 2, height // 2
                biggestObjectMiddle = ((x) * scaleFactor, (y) * scaleFactor)
                distanceVector = tuple(map(lambda x, y: x - y, biggestObjectMiddle, screenMiddle))
                # print("Vector: {}".format(distanceVector))
                scaled = (translate(distanceVector[0], -width // 2, width // 2),
                          translate(distanceVector[1], -height // 2, height // 2))
                # print("Vector scaled: {}".format(scaled))
                pitch = scaled[1]  # up-down
                yaw = scaled[0]  # left-right
                cv2.line(upscaledColor, screenMiddle, biggestObjectMiddle, (0, 0, 255))
                # print(yaw)
                packet = '<packet, {}, {}>'.format(yaw, pitch)
                packetBytes = bytes(packet, 'utf-8')
                # print("PPPPPPacket: {}".format(packet))

        cv2.imshow("video", frame)
        # cv2.imshow("roi", roi)
        cv2.imshow("mask", mask)

        modTolerances = False

    key = cv2.waitKey(1) & 0xFF

    loopEnd = time.time()
    # print("loop execution took {:3.2f}ms".format((loopEnd - loopStart) * 1000))

# cleanup
cv2.destroyAllWindows()
vs.stop()
