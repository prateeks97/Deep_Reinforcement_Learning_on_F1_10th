import rospy
from ackermann_msgs.msg import AckermannDriveStamped

from threading import Thread
import time
import argparse
import numpy as np

try:
    from geometry_msgs.msg import PoseStamped
except ImportError:
    pass

try:
    from car.sensors import Sensors
except ImportError:
    from sensors import Sensors

PUBLISHER_WAIT = 0.005

MAX_SPEED_REDUCTION = 8
STEERING_SPEED_REDUCTION = 8
BACKWARD_SPEED_REDUCTION = 8
LIGHTLY_STEERING_REDUCTION = 2.4
BACKWARD_SECONDS = 1.6

MAX_SPEED_REDUCTION_SIM = 3
STEERING_SPEED_REDUCTION_SIM = 3
BACKWARD_SPEED_REDUCTION_SIM = 3
LIGHTLY_STEERING_REDUCTION_SIM = 2.4
BACKWARD_SECONDS_SIM = 1.8
USE_RESET_INSTEAD_OF_BACKWARDS_SIM = False

class Drive():
    def __init__(self, sensors, is_simulator=False):
        self.is_simulator = is_simulator
        if not is_simulator:
            topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
            max_steering = 0.34
            self.max_speed_reduction = MAX_SPEED_REDUCTION
            self.steering_speed_reduction = STEERING_SPEED_REDUCTION
            self.backward_speed_reduction = BACKWARD_SPEED_REDUCTION
            self.lightly_steering_reduction = LIGHTLY_STEERING_REDUCTION
            self.backward_seconds = BACKWARD_SECONDS
        else:
            topic = "/drive"
            max_steering = 0.4189
            self.max_speed_reduction = MAX_SPEED_REDUCTION_SIM
            self.steering_speed_reduction = STEERING_SPEED_REDUCTION_SIM
            self.backward_speed_reduction = BACKWARD_SPEED_REDUCTION_SIM
            self.lightly_steering_reduction = LIGHTLY_STEERING_REDUCTION_SIM
            self.backward_seconds = BACKWARD_SECONDS_SIM
            self.reset_publisher = rospy.Publisher("/pose", PoseStamped, queue_size=0)
        self.max_speed = rospy.get_param("max_speed", 5)
        # self.wheel_base = rospy.get_param("wheel_base")
        
        self.max_steering = rospy.get_param("max_steering", max_steering)
        self.drive_publisher = rospy.Publisher(topic, AckermannDriveStamped, queue_size=0)
        self.sensors = sensors
        self.stop()
        process = Thread(target=self.drive_command_runner)
        process.daemon = True
        process.start()
        print("max_speed: ", self.max_speed, ", max_steering: ", self.max_steering)

    def forward(self):
        steer = 0
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer
    def backward(self):
        self.send_drive_command(-self.max_speed/self.backward_speed_reduction, 0)
    
    def stop(self):
        self.send_drive_command(0, 0)
########################################################   
    
    def right_1(self):
        steer = -self.max_steering*1/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def right_2(self):
        steer = -self.max_steering*2/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def right_3(self):
        steer = -self.max_steering*3/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def right_4(self):
        steer = -self.max_steering*4/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def right_5(self):
        steer = -self.max_steering*5/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def right_6(self):
        steer = -self.max_steering*6/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def right_7(self):
        steer = -self.max_steering
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

####################################################################
    def left_1(self):
        steer = self.max_steering*1/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def left_2(self):
        steer = self.max_steering*2/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def left_3(self):
        steer = self.max_steering*3/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def left_4(self):
        steer = self.max_steering*4/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def left_5(self):
        steer = self.max_steering*5/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def left_6(self):
        steer = self.max_steering*6/7
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

    def left_7(self):
        steer = self.max_steering
        vel = (self.max_speed/self.max_speed_reduction) - (6.64853667702) * (steer)**2
        self.send_drive_command(vel, steer)
        return steer

###########################################################
    # def lightly_right(self):
    #     self.send_drive_command(self.max_speed/self.steering_speed_reduction, -self.max_steering/self.lightly_steering_reduction)

    # def lightly_left(self):
    #     self.send_drive_command(self.max_speed/self.steering_speed_reduction, self.max_steering/self.lightly_steering_reduction)

    def send_drive_command(self, speed, steering_angle):
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = speed
        ack_msg.drive.steering_angle = steering_angle
        self.ack_msg = ack_msg

    def drive_command_runner(self):
        while True:
            self.drive_publisher.publish(self.ack_msg)
            time.sleep(PUBLISHER_WAIT)

    def backward_until_obstacle(self):
        if USE_RESET_INSTEAD_OF_BACKWARDS_SIM and self.is_simulator:
            self.reset_simulator()
        else:
            self.backward()
            start = time.time()
            while not self.sensors.back_obstacle() and time.time() - start < self.backward_seconds:
                time.sleep(0.01)
            self.stop()
            time.sleep(0.1)


    def reset_simulator(self):
        if self.is_simulator:
            self.reset_publisher.publish(PoseStamped())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", action='store_true', help="to set the use of the simulator")
    args = parser.parse_args()

    run_seconds = 0.3
    rospy.init_node('drive_test')
    drive = Drive(args.simulator)
    while True:
        print("Write command")
        cmd = input()
        start = time.time()
        if cmd == "w":
            while time.time() - start < run_seconds:
                drive.forward()
        if cmd == "s":
            while time.time() - start < run_seconds:
                drive.backward()
        if cmd == "a":
            while time.time() - start < run_seconds:
                drive.lightly_left()
        if cmd == "d":
            while time.time() - start < run_seconds:
                drive.lightly_right()
        if cmd == "aa":
            while time.time() - start < run_seconds:
                drive.left()
        if cmd == "dd":
            while time.time() - start < run_seconds:
                drive.right()
        if cmd == " ":
            while time.time() - start < run_seconds:
                drive.stop()
        if cmd == "buo":
            drive.backward_until_obstacle()
        if cmd == "q":
            exit()

