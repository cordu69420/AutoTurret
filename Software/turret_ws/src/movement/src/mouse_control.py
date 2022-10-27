#!/usr/bin/env python3.7
import rospy
import setproctitle
import time
from pynput.mouse import Listener
from std_msgs.msg import Float32MultiArray

class ControlLogic():
    """Class used to control the movement logic of the turret based on mouse or image processing.
    """
    def __init__(self) -> None:
        """Initialize the mouse listener class.
        """
        # VARIABLES
        self.current_x = None
        self.current_y = None
        self.click_applied = False
        self.listener = self.create_mouse_listener()
        time.sleep(3)
        # Turret movement range
        self.yaw_range = [0, 110] # degrees
        self.elevation_range = [70 ,130] # degrees
        # Monitor resolution range
        self.monitor_resolution_yaw = [0, 1920] # pixels
        self.monitor_resolution_elevation = [0, 1080] # pixels
        # Turret desired rotation
        self.target_yaw = None
        self.target_elevation = None
        # PUBLISHERS
        self.angle_publisher = rospy.Publisher("/movement/angles", Float32MultiArray, queue_size=1)
        self.publish_data()

    def on_move(self, x, y) -> None:
        """Callback for mouse movement.

        Args:
            x (float): x coordinate of the mouse
            y (float): y coordinate of the mouse
        """

        self.current_x = x
        self.current_y = y

    def on_click(self, x, y, button, pressed) -> None:
        """Callback for mouse click

        Args:
            x (float): x coordinate of the mouse
            y (float): y coordinate of the mouse
            button (string): the pressed button
            pressed (bool): is pressed
        """
        if pressed:
            self.click_applied = True


    def on_scroll(self, x, y, dx, dy):
        """Callback for mouse scroll. Irrelevant

        Args:
            x (float): _description_
            y (float): _description_
            dx (float): _description_
            dy (float): _description_
        """
        pass
            
    def create_mouse_listener(self) -> Listener:
        """Create a mouse listener class.

        Returns:
            Listener: mouse listener class initialized.
        """
        self.listener = Listener(
            on_move=self.on_move,
            on_click=self.on_click,
            on_scroll=self.on_scroll)
        self.listener.start()

        return self.listener
    
    def pixel_to_turret(self) -> None:
        """Convert the pixel position to a turret rotation.
        """
        # Linear function solution
        # f(x) = (d - c) * ((x-a)/(b-a) + c)
        if self.current_x is not None:
            self.target_yaw = (self.yaw_range[1] - self.yaw_range[0]) * ((self.current_x - self.monitor_resolution_yaw[0]) / (self.monitor_resolution_yaw[1] - self.monitor_resolution_yaw[0])) + self.yaw_range[0]
            self.target_elevation = (self.elevation_range[1] - self.elevation_range[0]) * ((self.current_y - self.monitor_resolution_elevation[0]) / (self.monitor_resolution_elevation[1] - self.monitor_resolution_elevation[0])) + self.elevation_range[0]

    def publish_data(self) -> None:
        """Send pixel data as rotation to the turret to achieve the desired movement
        """
        # Rate to control the loop frequency
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.pixel_to_turret()
            # Create the angle message
            msg = Float32MultiArray()
            msg.data = [self.target_yaw, self.target_elevation]
            self.angle_publisher.publish(msg)
            rate.sleep()
        # Stop the mouse listener
        self.listener.join()

if __name__ == '__main__':
    setproctitle.setproctitle('control_logic')
    rospy.init_node('control_logic')
    ControlLogic()
    rospy.spin()   