#!/usr/bin/env python

import rospy
import math
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from scipy import interpolate

# Constants
LIST_TREE = [(2,5), (7,7), (10,9)]
SKIP_LEFT_POINTS = [24, 25]
SKIP_RIGHT_POINTS = [11, 13, 39, 40, 41]
DISTANCE = 1.5

NUM_POINTS_BETWEEN_KEYPOINTS = 4  # definisci quanti segmenti di strada vuoi tra ciascuna coppia di keypoint

def interpolate_points(p1, p2, num_points):
    """Interpolates points between two given points."""
    x_values = np.linspace(p1[0], p2[0], num_points + 2)[1:-1]  # +2 e [1:-1] per escludere p1 e p2
    y_values = np.linspace(p1[1], p2[1], num_points + 2)[1:-1]
    return list(zip(x_values, y_values))

def read_file_sdf(filename):
    """Reads and returns the content of an SDF file."""
    with open(filename, 'r') as file:
        return file.read()

MODEL_CONTENT_CONE_LEFT = read_file_sdf(filename = 'cone_yellow/model.sdf')
MODEL_CONTENT_CONE_RIGHT = read_file_sdf(filename = 'cone_blue/model.sdf')
MODEL_CONTENT_ROAD = read_file_sdf(filename = 'road.sdf')  # Assuming you have an sdf named road_.sdf
MODEL_CONTENT_TREE = read_file_sdf(filename = 'tree.sdf')

def generate_trajectory():
    """Generates the primary trajectory using sin curve."""
    cx = np.arange(0, 50, 1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    tck = interpolate.splrep(cx, cy)
    return cx, cy, tck

def generate_track(cx, cy, tck):
    """Generates left and right points based on the primary trajectory."""
    points_list_left = []
    points_list_right = []

    for i in range(len(cx)):
        x0, y0 = cx[i], cy[i]
        dydx = interpolate.splev(x0, tck, der=1)
        angle = np.arctan(abs(dydx))
        
        if dydx >= 0:
            x1, y1 = x0 - DISTANCE * np.sin(angle), y0 + DISTANCE * np.cos(angle)
            x2, y2 = x0 + DISTANCE * np.sin(angle), y0 - DISTANCE * np.cos(angle)
        else:
            x1, y1 = x0 + DISTANCE * np.sin(angle), y0 + DISTANCE * np.cos(angle)
            x2, y2 = x0 - DISTANCE * np.sin(angle), y0 - DISTANCE * np.cos(angle)

        points_list_left.append((x1, y1))
        points_list_right.append((x2, y2))
    
    return points_list_left, points_list_right

def compute_middle_points(left_points, right_points):
    """Compute middle points between left and right points."""
    return [((l[0] + r[0]) / 2, (l[1] + r[1]) / 2) for l, r in zip(left_points, right_points)]

def spawn(name, x, y, z, model_content):
    """Spawns a model in the simulation environment."""
    spawn_model = rospy.ServiceProxy('steer_bot/gazebo/spawn_sdf_model', SpawnModel)
    
    initial_pose = Pose()
    initial_pose.position.x = x 
    initial_pose.position.y = y
    initial_pose.position.z = z 

    response = spawn_model(name, model_content, "", initial_pose, "world")
    rospy.loginfo(response.status_message)

def node():
    """ROS node to generate trajectories and spawn models."""
    rospy.init_node('spawn_model')
    rospy.wait_for_service('steer_bot/gazebo/spawn_sdf_model')

    cx, cy, tck = generate_trajectory()
    left_points, right_points = generate_track(cx, cy, tck)
    
    try:
        

        # Qui generiamo segmenti di strada aggiuntivi tra ogni coppia di keypoint
        for i in range(len(left_points) - 1):  # -1 poiché guardiamo al punto successivo
            start_point = ((left_points[i][0] + right_points[i][0]) / 2, (left_points[i][1] + right_points[i][1]) / 2)
            end_point = ((left_points[i + 1][0] + right_points[i + 1][0]) / 2, (left_points[i + 1][1] + right_points[i + 1][1]) / 2)

            middle_points = interpolate_points(start_point, end_point, NUM_POINTS_BETWEEN_KEYPOINTS)
            for j, (x, y) in enumerate(middle_points):
                spawn(f'roadSegment_{i}_{j}', x, y, 0, MODEL_CONTENT_ROAD)
        # Qui generiamo i coni che delimitano il percorso 
        for i, (x, y) in enumerate(left_points):
            if i not in SKIP_LEFT_POINTS:
                print('cono sinistra')
                spawn(f'pointL_{i}_x', x, y, 0.3, MODEL_CONTENT_CONE_LEFT)

        for i, (x, y) in enumerate(right_points):
            if i not in SKIP_RIGHT_POINTS:
                print('cono destra')
                spawn(f'pointR_{i}_x', x, y, 0.3, MODEL_CONTENT_CONE_RIGHT)
        # Qui generiamo gli alberi per decorazione
        i=0
        for tree in LIST_TREE:
            i += 1
            x, y = tree
            spawn("tree"+ str(i), x, y, 0, MODEL_CONTENT_TREE)
        

    except rospy.ServiceException as exc:
        rospy.logerr("Error during service invocation: %s", str(exc))

if __name__ == '__main__':
    node()
