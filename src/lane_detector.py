#!/usr/bin/env python
"""
This node finds lanes using a Canny edge detector and Hough transform.
It is intended to detect right and left lane borders always as straight lines. 
Curves are also detected as straight lines under the assumption that curvature
is small enough.
Detected lines are published in normal form (rho, theta) as 2-float arrays.
rho is measured in pixels and theta in radians
"""

import cv2
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

#
# Converts a line given by two points to the normal form (rho, theta) with
# rho, the distance to origin and theta, the angle wrt positive x-axis
#
def to_normal_form(x1, y1, x2, y2):
    A = y2 - y1
    B = x1 - x2
    C = A*x1 + B*y1
    theta = np.arctan2(B, A)
    rho   = C/np.sqrt(A*A + B*B)
    if rho < 0:
        theta += np.pi
        rho = -rho
    return np.asarray([rho, theta])

#
# In image coordinates, origin is in the top-left corner, nevertheless,
# for lane detection purposes, it is more useful to have coordinates taking
# as origin the center of the car-hood, which corresponds to the bottom-center
# point in the image. This function makes such transformation.
#
def translate_lines_to_bottom_center(lines, x_center, y_center):
    if lines is None:
        return None
    new_lines = []
    for x1, y1, x2, y2 in lines:
        nx1 = x1 - x_center
        nx2 = x2 - x_center
        ny1 = y_center - y1
        ny2 = y_center - y2
        new_lines.append([nx1, ny1, nx2, ny2])
    return new_lines

#
# Draws a line given in normal form, in coordinates wrt car's hood.
#
def draw_normal_line(rho, theta, length, img,color):
    if rho == 0 or theta == 0:
        return
    a  = np.cos(theta)
    b  = np.sin(theta)
    x1 = int(a*rho - b*length + img.shape[1]/2)
    y1 = int(img.shape[0] - (b*rho + a*length))
    x2 = int(a*rho + b*length + img.shape[1]/2)
    y2 = int(img.shape[0] - (b*rho - a*length))
    cv2.line(img, (x1, y1), (x2, y2), color, 3, cv2.LINE_AA)

#
# Convert to grayscale              
# Apply blur filter to reduce noise 
# Canny edge detector               
#
def detect_edges(img):
    global line_pub
    # bridge = CvBridge()
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    medBlur = cv2.medianBlur(gray, 5)
    __, th1 = cv2.threshold(medBlur, 99, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((5,5))
    closing = cv2.morphologyEx(th1, cv2.MORPH_CLOSE, kernel)
    # __, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    # kernel = np.ones((5,5))
    # opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel) 
    blur = cv2.GaussianBlur(gray, (5, 5), 0)       
    canny = cv2.Canny(closing, 50, 150)
    # output_image = bridge.cv2_to_imgmsg(canny, encoding = '8UC1')	
    # line_pub.publish(output_image)               
    return canny

#
# This function removes all lines with slopes near to horizontal al vertical lines.
# It returns lines in two sets: left-border lines and right-border lines.
# Classification is made based only on angle theta
#
def filter_lines(lines):
    left_lines  = []
    right_lines = []
    for x1, y1, x2, y2 in lines:
        rho, theta = to_normal_form(x1, y1, x2, y2)
        if (theta > -(np.pi/2-0.3) and theta < -0.1) or (theta > 0.1 and theta < (np.pi/2 - 0.3)):
            right_lines.append([x1, y1, x2, y2])
        if (theta > (np.pi/2 + 0.3) and theta < np.pi*0.9) or (theta > -0.9*np.pi and theta < -(np.pi/2 + 0.3)):
            left_lines.append([x1, y1, x2, y2])
    left_lines  = left_lines  if len(left_lines)  > 0 else None
    right_lines = right_lines if len(right_lines) > 0 else None
    return left_lines, right_lines

#
# Calculates a weighted average of all lines detected. Longer lines have greater weight.
# Average is calculated using lines in normal form. 
#
def weighted_average(lines):
    if lines is None or len(lines) == 0:
        return 0, 0
    weights = np.asarray([np.sqrt((x2 - x1)**2 + (y2 - y1)**2) for x1, y1, x2, y2 in lines])
    weights = weights/sum(weights)
    weighted_average = np.asarray([0.0,0.0])
    for i in range(len(lines)):
        x1, y1, x2, y2 = lines[i]
        rho, theta = to_normal_form(x1, y1, x2, y2)
        weighted_average[0] += rho*weights[i]
        weighted_average[1] += theta*weights[i]
    return weighted_average[0], weighted_average[1]

#
# Image callback. For detecting lane borders, the following steps are performed:
# - Crop image to consider only the part below horizon
# - Conversion to grayscale, gaussian filtering and Canny border detection
# - Detect lines from borders using Hough transform
# - Filter lines to keep only the ones which are more likely to be a lane border
# - Perform a weighted average
# - Publish detected lanes
#
def callback_rgb_image(msg):
    global pub_left_lane, pub_right_lane, image_pub
    bridge = CvBridge()
    img   = bridge.compressed_imgmsg_to_cv2(msg)
    img   = img[int(0.4*img.shape[0]):int(0.97*img.shape[0]) ,:,:]
    canny = detect_edges(img)
    lines = cv2.HoughLinesP(canny, 2, np.pi/180, 80, minLineLength=80, maxLineGap=100)[:,0]
    lines = translate_lines_to_bottom_center(lines, img.shape[1]/2, img.shape[0])
    left_lines, right_lines = filter_lines(lines)
    mean_rho_l, mean_theta_l = weighted_average(left_lines)
    mean_rho_r, mean_theta_r = weighted_average(right_lines)
    msg_left_lane  = Float64MultiArray()
    msg_right_lane = Float64MultiArray()
    msg_left_lane.data  = [mean_rho_l, mean_theta_l]
    msg_right_lane.data = [mean_rho_r, mean_theta_r]
    pub_left_lane.publish(msg_left_lane)
    pub_right_lane.publish(msg_right_lane)
    draw_normal_line(mean_rho_l, mean_theta_l, img.shape[0], img, (255,0,0))
    draw_normal_line(mean_rho_r, mean_theta_r, img.shape[0], img, (0,0,255))
    output_image = bridge.cv2_to_imgmsg(img, encoding = 'bgr8')	
    image_pub.publish(output_image)
    # cv2.imshow("Region of interest", img)
    # cv2.waitKey(10)

def main():
    global pub_left_lane, pub_right_lane, image_pub,line_pub
    print("INITIALIZING LANE DETECTION DEMO...")
    rospy.init_node("lane_detector")
    rospy.Subscriber('/app/camera/rgb/image_raw/compressed', CompressedImage, callback_rgb_image)
    pub_left_lane  = rospy.Publisher("/demo/left_lane" , Float64MultiArray, queue_size=10)
    pub_right_lane = rospy.Publisher("/demo/right_lane", Float64MultiArray, queue_size=10)
    image_pub = rospy.Publisher("/image/line_tracker", Image, queue_size = 1)
    line_pub = rospy.Publisher("/image/line_class", Image, queue_size = 1)
    rate = rospy.Rate(10)
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
        
    except:
        rospy.ROSInterruptException
        pass