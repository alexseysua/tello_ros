import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from tello_msgs.srv import TelloAction
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError  # Package to convert between ROS and OpenCV Images

import cv2
import cv2.aruco as aruco

from .Pid import PID


class ActionManager(Node):
    def __init__(self):
        Node.__init__(self, 'controller_manager')
        self.cli = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()

    def ask_for_takeoff(self):
        self.req.cmd = "takeoff"
        self.future = self.cli.call_async(self.req)

    def ask_for_landing(self):
        self.req.cmd = "land"
        self.future = self.cli.call_async(self.req)


class Controller(Node):
    def __init__(self):
        Node.__init__(self, 'controller')

        self.aruco_detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_6X6_250), aruco.DetectorParameters())    #aruco config

        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 10)   #publisher for velocity commands
        self.create_timer(0.5, self.cmd_vel_loop)   #cmd_vel_loop called each 0.5s

        # Init desired velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.v_yaw = 0.0

        self.image_subscription_ = self.create_subscription(Image, '/drone1/image_raw', self.image_callback, qos_profile=qos_profile_sensor_data)   #subscriber for image
        self.display_image = True
        
        self.br = CvBridge()    # Used to convert between ROS and OpenCV images


    def cmd_vel_loop(self):
        """
        I'm used to publish every 0.5 seconds the velocity command.
        You don't need to modify it, just modify vx, vy and v_yaw when you need it.
        """
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
        msg.angular.z = self.v_yaw
        self.cmd_vel_publisher_.publish(msg)


    def image_callback(self, data):
        """ Method called whenever a new image is published on /drone1/image topic """
        try:
            cv_image = self.br.imgmsg_to_cv2(data)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = self.aruco_detector.detectMarkers(gray)

            gray = aruco.drawDetectedMarkers(gray, corners)

            if len(corners) > 0:
                print("TAG SEEN %s"%ids)
                #TODO : Here, write your controller
                #From the corners information, set the right command speed
                #Like self.vx, self.vy, self.v_yaw, to control the UAV
            else:
                print("No tag seen..")
                #The UAV stays in the same pose, it doesn't move
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.v_yaw = 0.0

            # Display the resulting frame
            if self.display_image:
                cv2.imshow('frame', gray)
                cv2.waitKey(3)

        except CvBridgeError as e:
            pass


def main(args=None):
    rclpy.init(args=args)

    # Wait for take off service to be ready
    action_manager = ActionManager()
    action_manager.ask_for_takeoff()
    ready_to_continue_mission = False

    # Try to takeoff, wait for the return of the service
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Take off is a success !")
                    ready_to_continue_mission = True
                else:  # NOT OK
                    print("Something is wrong with the takeoff. LANDING NOW")
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    if ready_to_continue_mission:
        controller = Controller()
        try:
            while rclpy.ok():
                rclpy.spin_once(controller)
        except KeyboardInterrupt:
            print("Stopping the control. Ask for landing.")
        controller.destroy_node()

    # ASK FOR LANDING
    action_manager.ask_for_landing()
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Landing is a success !")
                    break  # Only if landing is ok
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    rclpy.shutdown()