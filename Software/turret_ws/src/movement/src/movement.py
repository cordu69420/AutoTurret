#!/usr/bin/env python3.7
import rospy
import setproctitle
import adafruit_servokit
import time
import RPi.GPIO as GPIO
import threading
# import cv2
from std_msgs.msg import Float32MultiArray, String, Bool
GPIO.cleanup()
class Movement():
    """Class used to define the movement of the turret.
    """
    def __init__(self) -> None:
        # VARIABLES
        # Breakout circuit pins for servos
        self.yaw_pin = 0
        self.elevation_pin = 1
        self.gpio_pin = 4
        # Adafruit breakout kit
        self.kit = adafruit_servokit.ServoKit(channels=16)
        # Starting position of the turret
        self.starting_elevation_angle = 25
        self.starting_yaw_angle = 55
        # Current position
        self.current_elevation_angle = 25
        self.current_yaw_angle = 55
        # Target angle from the inputs
        self.target_yaw = 55
        self.target_elevation = 25
        self.shoot_weapon = False
        self.movement_rate = 1
        self.yaw_lock = threading.Lock()
        self.elevation_lock = threading.Lock()
        # SUBSCRIERS
        self.direction_listener = rospy.Subscriber("/movement/direction", String, self.direction_callback)
        self.toggle_listener = rospy.Subscriber("/movement/shoot", Bool, self.toggle_callback)     
        # Initialize the turret
        self.init_turret()
        time.sleep(2)
        # Start the movement
        self.move_turret_angle()

    def init_turret(self) -> None:
        """Init the turret position the the established angles.
        """
        self.move_servo_yaw()
        self.move_servo_elevation()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.OUT, initial=GPIO.LOW)
    
    def move_servo_yaw(self) -> None:
        """Rotate the turret on the yaw axis by the given angle.
        """
        self.yaw_lock.acquire()
        self.kit.servo[self.yaw_pin].angle = self.target_yaw
        time.sleep(0.0018)
        self.current_yaw_angle = self.target_yaw
        self.yaw_lock.release()

    def move_servo_elevation(self) -> None:
        """Rotate the turret on the elevation axis by the given angle.
        """
        self.elevation_lock.acquire()
        self.kit.servo[self.elevation_pin].angle = self.target_elevation
        time.sleep(0.0018)
        self.current_elevation_angle = self.target_elevation
        self.elevation_lock.release()
        
    def direction_callback(self, msg) -> None:
        """Callback used to process the the given direction

        Args:
            msg (String): The direction
        """
        
        if msg.data == "down" and self.target_elevation >= self.movement_rate:
            self.elevation_lock.acquire()
            self.target_elevation -= self.movement_rate
            self.elevation_lock.release()
        if msg.data == "up" and self.target_elevation <= 55 - self.movement_rate:
            self.elevation_lock.acquire()
            self.target_elevation += self.movement_rate
            self.elevation_lock.release()
        
        
        if msg.data == "left" and self.target_yaw >= self.movement_rate:
            self.yaw_lock.acquire()
            self.target_yaw -= self.movement_rate
            self.yaw_lock.release()
        if msg.data == "right" and self.target_yaw <= 180 - self.movement_rate:
            self.yaw_lock.acquire()  
            self.target_yaw += self.movement_rate
            self.yaw_lock.release()
        
    def toggle_callback(self, msg) -> None:
        """Callback used to shoot the weapon

        Args:
            msg (_type_): _description_
        """
        self.shoot_weapon = True
        time.sleep(5)

    def move_turret_angle(self) -> None:
        """Main function to move the turret on the main thread.
        """
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            if self.target_elevation is not None and self.target_yaw is not None:
                self.move_servo_yaw()
                self.move_servo_elevation()
                # print(self.target_yaw)
                if self.shoot_weapon:
                    self.shoot_weapon = False
                    GPIO.output(self.gpio_pin, GPIO.HIGH)
                    time.sleep(0.18)
                    GPIO.output(self.gpio_pin, GPIO.LOW)
                rate.sleep()
        
        self.target_yaw = self.starting_yaw_angle
        self.target_elevation = self.starting_elevation_angle
        self.move_servo_yaw()
        self.move_servo_elevation()
        


if __name__ == '__main__':
    setproctitle.setproctitle('turret_movement')
    rospy.init_node('turret_movement')
    Movement()
    rospy.spin()
    GPIO.cleanup()
    # rospy.spin()