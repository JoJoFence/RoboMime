"""
Script Name: KinectJointOrientations.py
Author: Jonas Hansen
Affiliation: Human-Robot Interaction Laboratory, Kyoto University
Date: August 2, 2023
Description:
    This script tracks the joint orientations of a human using Microsoft's Azure Kinect camera. Currently,
    this script is adapted for converting the human's joint orientations to the Robovie robot's joint angles
    (which have a specific format and custom angle limitations). Specifically, this program tracks the body 
    skeleton of the closest person to the camera and has a series of functions to return the elbow, shoulder, 
    and head/neck angles in Robovie's roll, pitch, and yaw format.
Necessary Software:
    Be sure to install the following using pip:
    - OpenCV (pip install opencv-python)
    - numpy (pip install numpy)
    - scipy (pip instal scipy)
    - pykinect_azure (pip install pykinect_azure)
    Download the Azure Kinect Sensor SDK and Body Tracking SDK:
    - Sensor SDK: https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download
    - Body Tracking SDK: https://learn.microsoft.com/en-us/azure/kinect-dk/body-sdk-download
Necessary Hardware:
    - Microsoft Azure Kinect DK Camera (https://www.microsoft.com/en-us/d/azure-kinect-dk/8pp5vxmd9nhq)
"""

# First we import relevant packages
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import pykinect_azure as pykinect

###############################################
# --- Relevant joints for Robovie control! ---

# Left arm
left_clavicle = pykinect.K4ABT_JOINT_CLAVICLE_LEFT
left_shoulder = pykinect.K4ABT_JOINT_SHOULDER_LEFT
left_elbow = pykinect.K4ABT_JOINT_ELBOW_LEFT
left_wrist = pykinect.K4ABT_JOINT_WRIST_LEFT

# Right arm
right_clavicle = pykinect.K4ABT_JOINT_CLAVICLE_RIGHT
right_shoulder = pykinect.K4ABT_JOINT_SHOULDER_RIGHT
right_elbow = pykinect.K4ABT_JOINT_ELBOW_RIGHT
right_wrist = pykinect.K4ABT_JOINT_WRIST_RIGHT

# Head, nose, and neck
head = pykinect.K4ABT_JOINT_HEAD
nose = pykinect.K4ABT_JOINT_NOSE
neck = pykinect.K4ABT_JOINT_NECK

# Torso
chest = pykinect.K4ABT_JOINT_SPINE_CHEST
navel = pykinect.K4ABT_JOINT_SPINE_NAVEL

##############################################


##############################################
# --- Defining Robovie's (and some human) joint limits (in degrees) --- 

# Head roll limits
rv_head_roll_min = -50
rv_head_roll_max = 40

# Head pitch limits
rv_head_pitch_min = -40
rv_head_pitch_max = 40

# Neck yaw limits
rv_neck_yaw_min = -90
rv_neck_yaw_max = 90

# Shoulder pitch limits
rv_shoulder_pitch_min = -150
rv_shoulder_pitch_max = 150
human_shoulder_pitch_min = -75

# Shoulder roll limits
rv_shoulder_roll_min = -75
rv_shoulder_roll_max = 10
shoulder_roll_offset = 10 # Robovie's shoulder roll is slightly off from the default roll calculated here, so we adjust

# Elbow yaw limits
rv_elbow_yaw_min = -40
rv_elbow_yaw_max = 45
elbow_yaw_offset = 10 # Robovie's elbow yaw is slightly off from the default yaw calculated here, so we adjust 

# Elbow pitch limits
rv_elbow_pitch_min = 15
rv_elbow_pitch_max = 145

##############################################


def three_points_to_angle(a_pos, b_pos, c_pos):
    """
    DESCRIPTION:
        Calculates the 3D angle between three points based on their positions in 3D-space relative to the position
        of the camera.
    INPUTS:
        a_pos, b_pos, c_pos (lists): each are lists containg the 3d coordinates of the point position. They are 
        of the form [x, y, z], accesed by doing [pos.x, pos.y, pos.z].
    OUTPUTS:
        angle (float): the angle betweent points a_pos, b_pos, and c_pos.
    """
    # The position object is a dictionary, so we first convert to numpy arrays
    a_arr = np.array([a_pos.x, a_pos.y, a_pos.z])
    b_arr = np.array([b_pos.x, b_pos.y, b_pos.z])
    c_arr = np.array([c_pos.x, c_pos.y, c_pos.z])

    # Next we calculate the vectors
    ba_vec = a_arr - b_arr
    bc_vec = c_arr - b_arr

    cosine_angle = np.dot(ba_vec, bc_vec) / (np.linalg.norm(ba_vec) * np.linalg.norm(bc_vec))
    angle = np.degrees(np.arccos(cosine_angle))

    return angle


def get_joint_angle(body, joints):
    """
    DESCRIPTION:
        Using the three_points_to_angle() function, this function returns the central joint angle between three
        consecutive joints. For example, if you wanted the angle of the elbow joint, you would input the shoulder
        joint, the elbow joint, and wrist joint.
    INPUTS:
        body (object): pykinect_azure's 'body' object, defined as body_frame.get_body()
        joints (list): list of three joint names, as variables (not strings). 
    OUTPUTS:
        joint_angle (float): the angle of the central joint (i.e. angle between the three joints).
    """
    joint0_pos = body.joints[joints[0]].position
    joint1_pos = body.joints[joints[1]].position
    joint2_pos = body.joints[joints[2]].position

    joint_angle = three_points_to_angle(joint0_pos, joint1_pos, joint2_pos)

    return joint_angle


def get_euler_coords(body, joint):
    """
    DESCRIPTION:
        Converts the joint orientation quaternion (x, y, z, w) into Euler coordinates (x, y, z). 
    INPUTS:
        body (object): pykinect_azure's 'body' object, defined as body_frame.get_body()
        joint (joint name variable or joint index integer): the joint you want the Euler coordinates for, as a 
        variable name or as the corresponding integer index of the joint.
    OUTPUTS:
        x, y, z (floats): the roll, pitch, and yaw (not necessarily respectively) of the joint. 
    """
    orientation = body.joints[joint].orientation
    joint_rot = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
    rot_euler = joint_rot.as_euler("xyz", degrees=True)

    x = np.round(rot_euler[0], 1)
    y = np.round(rot_euler[1], 1)
    z = np.round(rot_euler[2], 1)

    return x, y, z


def get_elbow_angles(body, side):
    """
    DESCRIPTION:
        Gets the roll, pitch, and yaw of the human's elbows and converts it into Robovie's roll, pitch, 
        and yaw definitions.
    INPUTS:
        body (object): pykinect_azure's 'body' object, defined as body_frame.get_body()
        side (string): string indicating which elbow to get the angles for (so 'left' or 'right')
    OUTPUTS:
        rv_roll, rv_pitch, rv_yaw (floats): the roll, pitch, and yaw (in degrees) to be sent to Robovie
    """
    if side == "left":
        roll, pitch, yaw = get_euler_coords(body, left_elbow)
        left_shoulder_roll = get_euler_coords(body, left_shoulder)[0]

        rv_roll = roll
        rv_pitch = abs(get_joint_angle(body, joints=[left_shoulder, left_elbow, left_wrist]) - 180)
        rv_yaw = (-left_shoulder_roll - 90) + elbow_yaw_offset
    elif side == "right":
        roll, pitch, yaw = get_euler_coords(body, right_elbow)
        right_shoulder_roll = get_euler_coords(body, right_shoulder)[0]

        rv_roll = roll
        rv_pitch = abs(get_joint_angle(body, joints=[right_shoulder, right_elbow, right_wrist]) - 180)
        rv_yaw = (-(right_shoulder_roll - 90)) + elbow_yaw_offset
    else:
        raise ValueError("Whoops! Please indicate which elbow you want to calculate the angles for" 
                         "('left' or 'right', for example get_elbow_angles(body, 'left'))).")

    if rv_pitch < rv_elbow_pitch_min:
        rv_pitch = rv_elbow_pitch_min
    if rv_pitch > rv_elbow_pitch_max:
        rv_pitch = rv_elbow_pitch_max
    if rv_yaw < rv_elbow_yaw_min:
        rv_yaw = rv_elbow_yaw_min
    if rv_yaw > rv_elbow_yaw_max:
        rv_yaw = rv_elbow_yaw_max
    
    return rv_roll, rv_pitch, rv_yaw


def get_shoulder_angles(body, side):
    """
    DESCRIPTION:
        Gets the roll, pitch, and yaw of the human's shoulders and converts it into Robovie's roll, pitch, 
        and yaw definitions.
    INPUTS:
        body (object): pykinect_azure's 'body' object, defined as body_frame.get_body()
        side (string): string indicating which shoulder to get the angles for (so 'left' or 'right')
    OUTPUTS:
        rv_roll, rv_pitch, rv_yaw (floats): the roll, pitch, and yaw (in degrees) to be sent to Robovie
    """
    if side == "left":
        roll, pitch, yaw = get_euler_coords(body, left_shoulder)
        
        # Getting x-positions of left elbow and left shoulder
        left_elbow_x_pos = body.joints[left_elbow].position.x
        left_shoulder_x_pos = body.joints[left_shoulder].position.x
        
        # Getting y-positions of the left elbow and left shoulder (and negating to point the y-axis upwards instead)
        left_elbow_y_pos = -body.joints[left_elbow].position.y 
        left_shoulder_y_pos = -body.joints[left_shoulder].position.y

        # Calculating the left shoulder roll
        rv_roll = 1.67*get_joint_angle(body, joints=[left_clavicle, left_shoulder, left_elbow]) - 270

        if rv_roll < rv_shoulder_roll_min:
            rv_roll = rv_shoulder_roll_min
        if rv_roll > rv_shoulder_roll_max or (((left_elbow_y_pos - 25) > left_shoulder_y_pos) and (abs(left_elbow_x_pos - left_shoulder_x_pos) > 50)):
            rv_roll = rv_shoulder_roll_max

        if ((left_elbow_y_pos - 10) > left_shoulder_y_pos) and rv_roll < 5:
            rv_pitch = (155 - pitch)
        elif pitch < 0:
            rv_pitch = 1.3*pitch
        else:
            rv_pitch = pitch

        rv_yaw = yaw
    elif side == "right":
        roll, pitch, yaw = get_euler_coords(body, right_shoulder)
        
        # Getting x-positions of right elbow and and right shoulder
        right_elbow_x_pos = body.joints[right_elbow].position.x
        right_shoulder_x_pos = body.joints[right_shoulder].position.x
        
        # Getting y-positions of the right elbow and right shoulder (and negating to point the y-axis upwards instead)
        right_elbow_y_pos = -body.joints[right_elbow].position.y
        right_shoulder_y_pos = -body.joints[right_shoulder].position.y

        # Calculating the right shoulder roll
        rv_roll = 1.67*get_joint_angle(body, joints=[right_clavicle, right_shoulder, right_elbow]) - 270

        if rv_roll < rv_shoulder_roll_min:
            rv_roll = rv_shoulder_roll_min
        if rv_roll > rv_shoulder_roll_max or (((right_elbow_y_pos - 25) > right_shoulder_y_pos) and (abs(right_elbow_x_pos - right_shoulder_x_pos) > 50)):
            rv_roll = rv_shoulder_roll_max

        pitch = -pitch
        if ((right_elbow_y_pos - 10) > right_shoulder_y_pos) and rv_roll < 5:
            rv_pitch = (155 - pitch)
        elif pitch < 0:
            rv_pitch = 1.3*pitch
        else:
            rv_pitch = pitch

        rv_yaw = yaw
    else:
        raise ValueError("Whoops! Please indicate which shoulder you want to calculate the angles for" 
                         "('left' or 'right', for example get_shoulder_angles(body, 'left'))).")
    
    
    if rv_pitch < rv_shoulder_pitch_min:
        rv_pitch = rv_shoulder_pitch_min
    if rv_pitch > rv_shoulder_pitch_max:
        rv_pitch = rv_shoulder_pitch_max
    
    return rv_roll, rv_pitch, rv_yaw


def get_head_angles(body):
    """
    DESCRIPTION:
        Gets the roll, pitch, and yaw of the human's head and converts it into Robovie's roll, pitch, 
        and yaw definitions.
    INPUTS:
        body (object): pykinect_azure's 'body' object, defined as body_frame.get_body()
    OUTPUTS:
        rv_roll, rv_pitch, rv_yaw (floats): the roll, pitch, and yaw (in degrees) to be sent to Robovie
    """
    roll, pitch, yaw = get_euler_coords(body, head)
    
    rv_head_roll = -(yaw + 90)
    rv_head_pitch = pitch
    rv_neck_yaw = -(roll + 90)
    
    if rv_head_roll < rv_head_roll_min:
        rv_head_roll = rv_head_roll_min
    if rv_head_roll > rv_head_roll_max:
        rv_head_roll = rv_head_roll_max
    if rv_head_pitch < rv_head_pitch_min:
        rv_head_pitch = rv_head_pitch_min
    if rv_head_pitch > rv_head_pitch_max:
        rv_head_pitch = rv_head_pitch_max
    if rv_neck_yaw < rv_neck_yaw_min:
        rv_neck_yaw = rv_neck_yaw_min
    if rv_neck_yaw > rv_neck_yaw_max:
        rv_neck_yaw = rv_neck_yaw_max

    return rv_head_roll, rv_head_pitch, rv_neck_yaw


def print_joint_information(body, joint):
    """
    DESCRIPTION:
        Prints the roll, pitch, and yaw of a specified relevant joint (according to the roll, pitch, and yaw
        assigned in the Robovie system).
    INPUTS:
        body (object): pykinect_azure's 'body' object, defined as body_frame.get_body()
        joint (joint variable name): variable name of the joint you want to print the information for.
    """
    if joint == left_elbow:
        euler_coords = get_elbow_angles(body, "left")
    elif joint == right_elbow:
        euler_coords = get_elbow_angles(body, "right")
    elif joint == left_shoulder:
        euler_coords = get_shoulder_angles(body, "left")
    elif joint == right_shoulder:
        euler_coords = get_shoulder_angles(body, "right")
    elif joint == head:
        euler_coords = get_shoulder_angles(head)

    print("Robovie Orientation (Roll: {}, Pitch: {}, Yaw: {})".format(np.format_float_positional(euler_coords[0], precision=2), 
                                                                        np.format_float_positional(euler_coords[1], precision=2), 
                                                                        np.format_float_positional(euler_coords[2], precision=2)))


def initialize_pykinect():
    # Initialize the library, if the library is not found, add the library path as argument
    pykinect.initialize_libraries(track_body=True)

    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

    # Start device
    device = pykinect.start_device(config=device_config)
    
    # Start body tracker
    bodyTracker = pykinect.start_body_tracker()
    bodyTracker.set_temporal_smoothing = 1.0 # Smoothing factor to make skeleton less jittery (0 is no smoothing, 1 is max smoothing)

    return device, bodyTracker


def get_closest_person(body_frame):
    # For each body in the camera's view, 
    num_bodies = body_frame.get_num_bodies()
    body_dist_dict = {}
    for i in range(num_bodies):
        body = body_frame.get_body(i)
        if body.is_valid():
            navel_pos = body.joints[navel].position # Get depth to spine navel (joint index = 1)
            navel_dist = np.sqrt(navel_pos.x**2 + navel_pos.y**2 + navel_pos.z**2)
            body.id = i
            body_dist_dict[body] = navel_dist
            
    # Get the body of the closest person to the camera
    body = [key for key in body_dist_dict if body_dist_dict[key] == min(body_dist_dict.values())][0]

    return body


if __name__ == "__main__":

    device, bodyTracker = initialize_pykinect()

    cv2.namedWindow('Depth image with skeleton', cv2.WINDOW_NORMAL)
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
 
        num_bodies = body_frame.get_num_bodies()
        if num_bodies >= 1:
            # Get the closest body to the camera
            closest_body = get_closest_person(body_frame)

            # Draw the nearest person's skeleton
            combined_image = body_frame.draw_body2d(combined_image, closest_body.id)
        else:
            print("Hello? Anybody there? No bodies found...")
        
                
        # Overlay body segmentation on depth image
        cv2.imshow('Depth image with skeleton', combined_image)

        # Press q key to stop
        if cv2.waitKey(1) == ord('q'):  
            break
