#!/usr/bin/python
import argparse
import cv2
import numpy as np
import cv2
import math
import PID
import Image
import time

cap = cv2.VideoCapture(0)

pid = PID.PID()

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

def testPID(P=0.4,I=0.0,D=0.1,L=10):
    pid = PID.PID(P,I,D)


    pid.setSampleTime(0.01)
    End = L
    height, width, _ = frame.shape
    center_x = width / 2
    print center_x, "x"
    center_y = height / 2
    setpoint = pid.SetPoints(int(center_x))
    feedback = x_value
    time_list = []
    feedback_list = []
    time_list = []
    setpoint_list = []
    for i in range(1, End):
        tX = [60, 310]
        lx0 = [300, 50]
        lx1 = [330, 600]
        lf = np.interp([translation_vector[2, 0]], tX, lx0)
        lr = np.interp([translation_vector[2, 0]], tX, lx1)
        print int(lf[0]), "lf"
        print int(lr[0]), "lr"
        cv2.line(frame, (int(lf[0]), 0), (int(lf[0]), 480), (0, 255, 0), 2)
        cv2.line(frame, (int(lr[0]), 0), (int(lr[0]), 480), (0, 255, 0), 2)
        feed = pid.update(feedback)
        #print feed, "feed"
        if ((320 + feed) > lf) and ((320 + feed) < lr):
            print("<<<<<<--------Go Straight------>>>>>")
            status = ">>>>>>>"
        elif feed > 0:
            print ("right------>>>>>")
        elif feed < 0:
            print ("<<<<<-----Left")
            status= "<<<<<<<"
            time.sleep(0.02)
            
           
            
                
                    
        

# To draw Arco marker to represent axis
def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


# keep looping
# try:
x_value = 0

while True:

    # grab the current frame and initialize the status text
    (grabbed, frame) = cap.read()
    status = "No Targets"

    # check to see if we have reached the end of the
    # video
    if not grabbed:
        break

    # convert the frame to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    edged = cv2.Canny(blurred, 50, 150)

    # find contours in the edge map
    # print cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    (_, cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    Area = 0

    # loop over the contours
    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)

        # ensure that the approximated contour is "roughly" rectangular
        if len(approx) >= 4 and len(approx) <= 6:
            # compute the bounding box of the approximated contour and
            # use the bounding box to compute the aspect ratio
            (x1, y1, w, h) = cv2.boundingRect(approx)
            # print x1
            # print y1

            aspectRatio = w / float(h)

            # compute the solidity of the original contour
            Area = []
            area = cv2.contourArea(c)
            Area = max(area, Area)
            # print Area, "area"
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area / float(hullArea)

            # compute whether or not the width and height, solidity, and
            # aspect ratio of the contour falls within appropriate bounds
            keepDims = w > 25 and h > 25
            keepSolidity = solidity > 0.9
            keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2

            # ensure that the contour passes all our tests
            if keepDims and keepSolidity and keepAspectRatio:

                # draw an outline around the target and update the status
                # text
                cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)
                status = "Target(s) Acquired"
                # This will give you the Pixel location of the rectangular box

                rc = cv2.minAreaRect(approx[:])

                box = cv2.boxPoints(rc)
                pt = []
                for p in box:
                    val = (p[0], p[1])
                    # print val
                    pt.append(val)
                    # print pt,'pt'
                    # cv2.circle(frame, val, 5, (200, 0, 0), 2)

                # print 'came to 2'
                M = cv2.moments(approx)
                (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                (startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
                (startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
                cv2.line(frame, (startX, cY), (endX, cY), (0, 255, 0), 3)
                cv2.line(frame, (cX, startY), (cX, endY), (0, 255, 0), 3)
                # print cX, "cxxx"
                # print cY, "cyyyy"
                x_value = cX
                print x_value,'cX'
                # print x_value, "xfierst"



                # perspective
                rows, cols, ch = frame.shape

                pts1 = np.float32([pt[1], pt[0], pt[2], pt[3]])
                # print pts1
                pts2 = np.float32([[0, 0], [640, 0], [0, 480], [640, 480]])
                M = None

                # try:
                M = cv2.getPerspectiveTransform(pts1, pts2)
                # except:
                #    print "Perspective failed"

                # print M

                ret, mtxr, mtxq, qx, qy, qz = cv2.RQDecomp3x3(M)
                # print qx
                C = M[2, 1] * M[2, 2]
                D = M[1, 0] * M[0, 0]
                Theta_xp = math.degrees(math.atan(C) * 2)
                # Theta_y = math.degrees(math.atan(C)*2)
                Theta_zp = math.degrees(math.atan(D) * 2)
                # print Theta_zp,'z'
                # print Theta_xp,'x'
                dst = cv2.warpPerspective(frame, M, (640, 480))

                # 2D image points. If you change the image, you need to change vector
                image_points = np.array([pt], dtype="double")
                # print image_points
                size = frame.shape
                # 3D model points.
                model_points = np.array([
                    (0.0, 0.0, 0.0),  # Rectangle center
                    (0.0, 13.6, 0.0),  #
                    (13.6, 13.6, 0.0),  #
                    (13.6, 0.0, 0.0),  #

                ])

                # Camera intrinsic parameters

                focal_length = size[1]
                center = (size[1] / 2, size[0] / 2)
                camera_matrix = np.array(
                    [[focal_length, 0, center[0]],
                     [0, focal_length, center[1]],
                     [0, 0, 1]], dtype="double"
                )

                # print "Camera Matrix :\n {0}".format(camera_matrix)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

                dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
                (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix,
                                                                              dist_coeffs, criteria)

                rotationMatrix = np.zeros((3, 3), np.float32)

                # print "Rotation Vector:\n {0}".format(rotation_vector)
                # print "Translation Vector:\n {0}".format(translation_vector)

                # Rodrigues to convert it into a Rotation vector

                dst, jacobian = cv2.Rodrigues(rotation_vector)
                # print "Rotation matrix:\n {0}".format(dst)
                # sy = math.sqrt(dst[0, 0] * dst[0, 0] + dst[1, 0] * [1, 0])
                A = dst[2, 1] * dst[2, 2]
                B = dst[1, 0] * dst[0, 0]
                # C = -dst[2,0]*sy



                # Eular angles

                Theta_x = math.degrees(math.atan(A) * 2)
                # Theta_y = math.degrees(math.atan(C)*2)
                Theta_z = math.degrees(math.atan(B) * 2)

                # print "Roll axis:\n {0}".format(Theta_x)
                # print "Yaw axis:\n {0}".format(Theta_z)


                (img_pts, jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector,
                                                        translation_vector, camera_matrix, dist_coeffs)
                # print img_pts

                # draw(frame,corners=pt,imgpts = img_pts)


                # if translation_vector[2,0]>160 and translation_vector[2,0]<180:
                #    print "go straight"
                #    cv2.line(frame,(200,0),(200,480),(0,255,0),2)
                #    cv2.line(frame, (440, 0), (440, 480), (0, 255, 0), 2)
                # print translation_vector[2,0]
                testPID(1.0, 1.0, 0)

    # draw the status text on the frame
    cv2.putText(frame, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 0), 2)
    # cv2.line(frame, (320, 0), (320, 480), (255, 0, 0), 2)
    # cv2.line(frame,(260,0),(260,480),(0,255,0),2)
    # cv2.line(frame, (380, 0), (380, 480), (0, 255, 0), 2)




    # show the frame and record if a key is pressed
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
        # print Area
# cleanup the camera and close any open windows
cv2.release()
cv2.destroyAllWindows()

