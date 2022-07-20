import cv2
import numpy as np
import logging
import math
import datetime
import sys

#import RPi_robot_control as control_robot

display_image = True


def detect_lane(frame):
    """
    This function is used for processing the current frame, detecting the Blue lane lines and generating the lanes image
    1st step: Detect the edges within only half of the screen frame.
    2nd step: From the edges, detect the line segments.
    3nd step: Generate the image with the detected lane lines.
    """
    logging.debug('detecting lane lines...')

    edges = detect_edges(frame)
    show_image('edges', edges)

    cropped_edges = define_region_of_interest(edges)
    show_image('cropped edges', cropped_edges)

    line_segments = detect_line_segments(cropped_edges)
    line_segment_image = display_lines(frame, line_segments)
    show_image("line segments", line_segment_image)

    lane_lines = find_average_slope_and_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    show_image("lane lines", lane_lines_image)

    return lane_lines, lane_lines_image

def detect_edges(frame):
    """
    This function is used for detecting the blue coloured edges, which are the lanes.
    For this we will use the HSV color parameter as it is is easier to represent a color in HSV than in RGB.
    At the end we use the Canny mask for the edge detection based on the range specified.
    """

    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    show_image("hsv", hsv)
    lower_blue_range = np.array([89, 50, 50])
    upper_blue_range = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue_range, upper_blue_range)
    show_image("blue mask", mask)

    # detect edges
    edges = cv2.Canny(mask, 100, 200)  # or (mask,200,400)

    return edges

def define_region_of_interest(canny):
    """
    This function is used for defining the region of interest for the lane detection. Therefore we focus only on the
    bottom half of the screen, as it is easier to control the robot with information only from half screen.
    """
    height, width = canny.shape
    mask = np.zeros_like(canny)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    show_image("mask", mask)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image

def detect_line_segments(cropped_edges):
    """
    This function is used for detecting the line segments by applying the Probabilistic Hough Line Transform. Some
    parameters like min_threshold, minLineLength, maxLineGap is a trial and error process, until reaching good
    parameters.
    It returns [(x1,y1),(x2,y2)] of the detected line segments.
    """
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                    maxLineGap=4)

    if line_segments is not None:
        for line_segment in line_segments:
            logging.debug('detected line_segment:')
            logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments

def find_average_slope_and_intercept(frame, line_segments):
    """
    This function checks every line segment and based on its slope, it is possible to state whether the line is on
    the left or right part of the screen. Thus it will combine all the line segments into left or right based on the
    defined boundary, in this case 1/3 of the screen's width.
    If a line segment slope is < 0, then we have detected left lane;
    If a line segment slope is > 0, then we have detected right lane.
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        control_robot.go_straight_with_speed(0)
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1 / 3
    right_region_boundary = width * (1 - boundary)  # right lane line segment should be on right (2/3 of the screen)
    left_region_boundary = width * boundary  # left lane line segment should be on left (1/3 of the screen)

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                # Here we skip vertical line, once its slope is infinite
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))
    logging.debug('lane lines: %s' % lane_lines)
    return lane_lines

def compute_heading_angle(frame, lane_lines):
    """
    This function calculates the angle on which the robot has to be heading to in order to be on the middle of the path.
    It receives the detect lane lines edges, and then calculates the heading angle.
    If there is only one lane detect in the current frame, then robot follows exactly this lane.
    """
    print(lane_lines)
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90  # return 0

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line and just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        # defining error margin for camera being on center.
        # 0.0 means car in direction to center, -0.03: car is in direction to left, +0.03 means car direction to right
        camera_mid_offset_percent = 0.02
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # finding the heading angle, which is angle between navigation direction and center line
    y_offset = int(height / 2)
    angle_to_mid_radian = math.atan(x_offset / y_offset)  # math.atan gives the angle in radian
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle converted in degrees to center vertical line
    # 90 is when the robot points to center, and the angle will be the deviation from the current robot's heading that's
    # why we add the 90 degrees.
    heading_angle = 90 - angle_to_mid_deg
    print(heading_angle)
    logging.debug('new heading angle: %s' % heading_angle)

    return heading_angle

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    """
    This is a helper function for displaying the line to the screen.
    Here we can modify the color and also line width.
    """
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, heading_angle, line_color=(0, 0, 255), line_width=5, ):
    """
    This function is used to display the heading line based on the heading_angle sent as argument.
    Here we can modify the color and also line width.
    """
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    heading_angle_radian = heading_angle / 180.0 * math.pi

    if heading_angle_radian == 0:  # if it is zero it will throw an error, so let's average it to 0.5
        heading_angle_radian = 0.5

    # (x1,y1) is always center bottom of the screen
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 + height / (2 * math.tan(heading_angle_radian)))  #Check calculations
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def length_of_line_segment(line):
    """
    This is a helper function to calculate the length of a line segment by using pythagoras theorem.
    """
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def show_image(title, frame, show=display_image):
    """
    This is a helper function that will help to display all the images generated throughout the image processing.
    It will show the images in case display_image is set to True.
    """
    if show:
        cv2.imshow(title, frame)


def make_points(frame, line):
    """
    This function is used to generate points [(x1,y1),(x2,y2)] based on the line parameters sent (slope and intercept)
    """
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # this is the bottom of the frame
    y2 = int(y1 * 1 / 2)  # the points will be made only from middle of the frame towards down

    # bound the coordinates within the frame
    # x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    # x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [[x1, y1, x2, y2]]


