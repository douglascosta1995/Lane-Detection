import logging
import RPi_robot_control as control_robot
import cv2
import datetime
from line_following_algorithm import LaneFollower

display_image = True


class AutoRobot(object):
    #initial parameters of the robot
    initial_speed = 0
    width_screen = 320
    height_screen = 240

    def __init__(self):
        """
        Here we initialize camera and robot
        """
        logging.info('Set up the robot...')

        control_robot.setup()

        logging.debug('Setting up camera')
        self.camera = cv2.VideoCapture(-1)
        self.camera.set(3, self.width_screen)
        self.camera.set(4, self.height_screen)

        logging.debug('Setting up initial speed of the robot')
        self.speed = control_robot.go_straight_with_speed(0)
        self.turn_right = control_robot.turn_right(0)
        self.turn_left = control_robot.turn_left(0)

        self.lane_follower = LaneFollower(self)

        logging.info('Created an Autonomous Robot')

    def create_video_recorder(self, path):
        """
        This function is used for recording the video during execution.
        """
        return cv2.VideoWriter(path, self.fourcc, 20.0, (self.width_screen, self.height_screen))

    def __enter__(self):
        """
        Entering a with statement
        """
        return self

    def __exit__(self, _type, value, traceback):
        """
        Exit a with statement
        """
        if traceback is not None:
            # If any exception occurred:
            logging.error('Exiting with statement with exception %s' % traceback)

        self.cleanup()

    def cleanup(self):
        """
        Stop the robot and reset to original state
        """
        logging.info('Stopping the robot.')
        control_robot.stop()
        self.camera.release()
        cv2.destroyAllWindows()

    def drive(self, speed=initial_speed):
        """
        This is the entry point of the control of the robot. When it is called, we set the robot to drive mode.
        It needs as parameter, the speed on which I want the robot to move. (0: stop - 100: max speed)
        """
        i = 0
        logging.info('Robot starting to drive at speed %s...' % speed)
        while self.camera.isOpened():
            _, image_lane = self.camera.read()
            i += 1
            # here if I want to record the video:
            # self.video_orig.write(image_lane)
            image_lane = self.follow_lane(image_lane)
            #self.video_lane.write(image_lane)
            show_image('Lane Following', image_lane)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cleanup()
                break

    def follow_lane(self, image):
        """
        It calls the follow_lane attribute from the LaneFollower
        """
        image = self.lane_follower.follow_lane(image)
        return image


def show_image(title, frame, show=display_image):
    if show:
        cv2.imshow(title, frame)


def main():
    with AutoRobot() as robot:
        robot.drive(20)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format='%(levelname)-5s:%(asctime)s: %(message)s')

    main()