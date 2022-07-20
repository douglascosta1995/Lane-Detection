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

def detect_edges(frame):
    """
    This function is used for detecting the blue coloured edges, which are the lanes.
    For this we will use the HSV color parameter as it is is easier to represent a color in HSV than in RGB.
    At the end we use the Canny mask for the edge detection based on the range specified.
    """


def define_region_of_interest(canny):
    """
    This function is used for defining the region of interest for the lane detection. Therefore we focus only on the
    bottom half of the screen, as it is easier to control the robot with information only from half screen.
    """


def detect_line_segments(cropped_edges):
    """
    This function is used for detecting the line segments by applying the Probabilistic Hough Line Transform. Some
    parameters like min_threshold, minLineLength, maxLineGap is a trial and error process, until reaching good
    parameters.
    It returns [(x1,y1),(x2,y2)] of the detected line segments.
    """

def find_average_slope_and_intercept(frame, line_segments):
    """
    This function checks every line segment and based on its slope, it is possible to state whether the line is on
    the left or right part of the screen. Thus it will combine all the line segments into left or right based on the
    defined boundary, in this case 1/3 of the screen's width.
    If a line segment slope is < 0, then we have detected left lane;
    If a line segment slope is > 0, then we have detected right lane.
    """


def compute_heading_angle(frame, lane_lines):
    """
    This function calculates the angle on which the robot has to be heading to in order to be on the middle of the path.
    It receives the detect lane lines edges, and then calculates the heading angle.
    If there is only one lane detect in the current frame, then robot follows exactly this lane.
    """


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    """
    This is a helper function for displaying the line to the screen.
    Here we can modify the color and also line width.
    """


def display_heading_line(frame, heading_angle, line_color=(0, 0, 255), line_width=5, ):
    """
    This function is used to display the heading line based on the heading_angle sent as argument.
    Here we can modify the color and also line width.
    """


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


