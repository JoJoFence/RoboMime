"""
Script Name: SendAnglesToRobovie.py
Author: Jonas Hansen
Affiliation: Human-Robot Interaction Laboratory, Kyoto University
Date: August 2, 2023
Description:
    This script sends the relevant joint angles tracked from the KinectJointOrientations.py script to the Robovie
    robot. There are a few options: you can send to the actual robot, you can send to the Robovie simulator, or you 
    can print the angles out as stdout.
Necessary Software:
    Be sure to install the following using pip:
    - OpenCV (pip install opencv-python)
    - numpy (pip install numpy)
    - scipy (pip install scipy)
    - keyboard (pip install keyboard)
    - pykinect_azure (pip install pykinect_azure)
    Download the Azure Kinect Sensor SDK and Body Tracking SDK:
    - Sensor SDK: https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download
    - Body Tracking SDK: https://learn.microsoft.com/en-us/azure/kinect-dk/body-sdk-download
Necessary Hardware:
    - Microsoft Azure Kinect DK Camera (https://www.microsoft.com/en-us/d/azure-kinect-dk/8pp5vxmd9nhq)

Usage:
    python SendAnglesToRobovie (-a or --actual [ROBOT_IP_ADDRESS] [DELAY] : sends angle data to the actual Robovie robot 
                                                                            and you replace [ROBOT_IP_ADDRESS] with the
                                                                            Robovie's IP address and [DELAY] with the 
                                                                            delay time between each send (in seconds).
                                -s or --sim : sends angle data to the Robovie simulator
                                -o or --output : prints the angle data as output
                                -c or --camera : displays what the Kinect Azure camera is seeing in real time
                                -h or --help : displays this information as a help message)
"""

# Importing the relevant packages
import cv2
import time
import sys
import argparse
import keyboard
import numpy as np
import pykinect_azure as pykinect

from kinect_joint_orientations import get_elbow_angles, get_shoulder_angles, get_head_angles, initialize_pykinect, get_closest_person
from motion_visualizer_updater import MotionVisualizerUpdater
from robot_motion_updater import RobotMotionUpdater

# Setup argparser
parser = argparse.ArgumentParser()
parser.add_argument("-a", "--actual", nargs=2, metavar=("[ROBOT_IP_ADDRESS]", "[DELAY]"),
                    help="Send angle data to actual Robovie (usage: -a [ROBOT_IP_ADDRESS] [DELAY])", default=False)
parser.add_argument("-s", "--sim", help="Send angle data to simulated Robovie", default=False, action="store_true")
parser.add_argument("-o", "--output", help="Print out angle data as output", default=False, action="store_true")
parser.add_argument("-c", "--camera", help="Displays what the Kinect Azure camera sees", default=False, action="store_true")
args = parser.parse_args(args=None if sys.argv[1:] else ['--help'])


def get_all_angles(body):
    # Left elbow angles
    left_elbow = get_elbow_angles(body, side="left")
    elbow_pitch_l = left_elbow[1]
    elbow_yaw_l = left_elbow[2]

    # Right elbow angles
    right_elbow = get_elbow_angles(body, side="right")
    elbow_pitch_r = right_elbow[1]
    elbow_yaw_r = right_elbow[2]

    # Left shoulder angles
    left_shoulder = get_shoulder_angles(body, side="left")
    shoulder_roll_l = left_shoulder[0]
    shoulder_pitch_l = left_shoulder[1]

    # Right shoulder angles
    right_shoulder = get_shoulder_angles(body, side="right")
    shoulder_roll_r = right_shoulder[0]
    shoulder_pitch_r = right_shoulder[1]

    # Head/neck angles
    head_angles = get_head_angles(body)
    head_roll = head_angles[0]
    head_pitch = head_angles[1]
    neck_yaw = head_angles[2]

    # Store all angles in a dictionary
    angle_dict = {"elbow_pitch_l": elbow_pitch_l, "elbow_yaw_l": elbow_yaw_l,
                  "elbow_pitch_r": elbow_pitch_r, "elbow_yaw_r": elbow_yaw_r,
                  "shoulder_roll_l": shoulder_roll_l, "shoulder_pitch_l": shoulder_pitch_l,
                  "shoulder_roll_r": shoulder_roll_r, "shoulder_pitch_r": shoulder_pitch_r,
                  "head_roll": head_roll, "head_pitch": head_pitch, "neck_yaw": neck_yaw}
    

    return angle_dict


if __name__ == "__main__":
    # Start up the Kinect Azure camera and initialize the pykinect instance
    device, bodyTracker = initialize_pykinect()

    if args.actual:
        # Start connection to actual Robovie
        robovie_connection = RobotMotionUpdater()
        robot_ip = str(args.actual[0])
        robovie_connection.connect(robot_ip)
    if args.sim:
        # Start connection to Robovie simulator
        sim_connection = MotionVisualizerUpdater()
        sim_connection.connect( "127.0.0.1" )

    if args.camera:
        # Open an OpenCV window to later displayq what the camera sees
        cv2.namedWindow('Depth image with skeleton', cv2.WINDOW_NORMAL)
   
    i = 0
    while True:
        # Get capture
        capture = device.update()

        # Get body tracker frame
        body_frame = bodyTracker.update()

        # Get the color depth image from the capture
        ret_depth, depth_color_image = capture.get_colored_depth_image()        
        depth_color_image = cv2.cvtColor(depth_color_image, cv2.COLOR_BGR2GRAY)

        # Get the colored body segmentation
        ret_color, body_image_color = body_frame.get_segmentation_image()
        body_image_color = cv2.cvtColor(body_image_color, cv2.COLOR_BGR2GRAY)

        if not ret_depth or not ret_color:
            continue
            
        # Combine both images
        combined_image = cv2.addWeighted(depth_color_image, 0.6, body_image_color, 0.4, 0)
 
        # Tracking only the closest person to the camera
        num_bodies = body_frame.get_num_bodies()
        if num_bodies >= 1:
            # Get the closest body to the camera
            closest_body = get_closest_person(body_frame)

            # Get the person's relevant joint angles
            angles = get_all_angles(closest_body)
            
            # Smoothing the transitions between angles and to reduce jitter
            if i == 0:
                prev_angles = angles

            min_threshold = 1.7
            max_threshold = 40
            for key in angles.keys():
                prev_angle = prev_angles[key]
                curr_angle = angles[key]
                if (min_threshold < abs(curr_angle - prev_angle) < max_threshold):
                    angles[key] = np.average([curr_angle, prev_angle])
                else:
                    angles[key] = prev_angle

            prev_angles = angles

            if args.actual and robovie_connection.isValid():
                # Sending the joint angles to Robovie
                robovie_connection.setAngle(elbow_pitch_l = angles["elbow_pitch_l"], elbow_yaw_l = angles["elbow_yaw_l"], 
                                elbow_pitch_r = angles["elbow_pitch_r"], elbow_yaw_r = angles["elbow_yaw_r"],
                                shoulder_roll_l = angles["shoulder_roll_l"], shoulder_pitch_l = angles["shoulder_pitch_l"],
                                shoulder_roll_r = angles["shoulder_roll_r"], shoulder_pitch_r = angles["shoulder_pitch_r"],   
                                head_roll = angles["head_roll"], head_pitch = angles["head_pitch"], neck_yaw = angles["neck_yaw"], 
                                duration=args.actual[1])
            if args.sim and sim_connection.isValid():
                # Sending the joint angles to Robovie simulator
                sim_connection.send(elbow_pitch_l = angles["elbow_pitch_l"], elbow_yaw_l = angles["elbow_yaw_l"], 
                                elbow_pitch_r = angles["elbow_pitch_r"], elbow_yaw_r = angles["elbow_yaw_r"],
                                shoulder_roll_l = angles["shoulder_roll_l"], shoulder_pitch_l = angles["shoulder_pitch_l"],
                                shoulder_roll_r = angles["shoulder_roll_r"], shoulder_pitch_r = angles["shoulder_pitch_r"],   
                                head_roll = angles["head_roll"], head_pitch = angles["head_pitch"], neck_yaw = angles["neck_yaw"])
            if args.output:
                # Print out the joint angle dictionary
                print(angles)

            # Draw the skeletons
            combined_image = body_frame.draw_body2d(combined_image, closest_body.id)
            i += 1
        else:
            print("Hello? Anybody there? No bodies found...")
        
        if args.actual:
            # Update RobotMotionUpdater connection (connection to actual Robovie)
            robovie_connection.update()
        if args.sim:
            # Update MotionVisualizerUpdater connection (connection to Robovie simulator)
            sim_connection.update()

        if args.camera:
            # Overlay body segmentation on depth image
            cv2.imshow('Depth image with skeleton', combined_image)

        # Press q key to stop
        if cv2.waitKey(1) == ord('q') or keyboard.is_pressed("q"):  
            break