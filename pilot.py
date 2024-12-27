
# import setup_path
import airsim.airsim as air_sim
from airsim.airsim.types import Pose, Vector2r, Vector3r

import numpy as np
import time
from utils import *
import cv2


def target_setup(drone:air_sim.MultirotorClient, seed=None, x=None, y=None):
    np.random.seed(seed)
    if x is None or not -55 < x < 30:
        x_val = np.random.uniform(-55, 30)
    else:
        x_val = x
    if y is None or not -160 < y < -75:
        y_val = np.random.uniform(-160, -75)
    else:
        y_val = y

    drone.simSetObjectPose('Target', Pose(get_position(x_val,y_val,1), get_orientation(pitch = np.pi/2)))
    drone.simSetObjectScale('Target', Vector3r(2,20,20))



def hoop_race(drone:air_sim.MultirotorClient, **kwargs):
    moveToPosition(drone, Vector3r(25, 0, -9))
    moveToPosition(drone, Vector3r(57, -31, -8))
    moveToPosition(drone, Vector3r(37, -31, -8))
    moveToPosition(drone, Vector3r(31, -24, -8))
    moveToPosition(drone, Vector3r(57, 31, -8))
    moveToPosition(drone, Vector3r(33, 34, -8))
    moveToPosition(drone, Vector3r(33, 28, -8))
    moveToPosition(drone, Vector3r(57, 0, -8))
    moveToPosition(drone, Vector3r(85, 0, -6))
    moveToPosition(drone, Vector3r(85, 0, -4))
    


def false_wall(drone:air_sim.MultirotorClient, **kwargs):
    moveToPosition(drone, Vector3r(120, 0, -4))
    moveToPosition(drone, Vector3r(120, -15, -4))
    moveToPosition(drone, Vector3r(135, -15, -4))
    moveToPosition(drone, Vector3r(135, -30, -4))
    moveToPosition(drone, Vector3r(135, -45, -4))
    moveToPosition(drone, Vector3r(120, -45, -4))
    moveToPosition(drone, Vector3r(105, -45, -4))
    moveToPosition(drone, Vector3r(105, -75, -4))
    turn_left(drone, 90)

    

    


def distance_maze(drone:air_sim.MultirotorClient, **kwargs):
    straight = drone.getDistanceSensorData('DistanceStraight').distance
    left = drone.getDistanceSensorData('DistanceLeft').distance
    right = drone.getDistanceSensorData('DistanceRight').distance
    width = (right + left)/2

    print(f'Straight ahead: {straight}')
    print(f'To left: {left}')
    print(f'To right: {right}')
    print(f"Corridor width: {width}")

    def fly_to_wall(drone, stop_distance):
        loop = True
        straight = drone.getDistanceSensorData('DistanceStraight').distance
        while loop:
            moveByBodyVelocityAndZ(drone, straight*0.25, 0, -4, duration = 0.5)
            straight = drone.getDistanceSensorData('DistanceStraight').distance
            if straight <= stop_distance:
                loop = False
        time.sleep(2)
    def fly_to_left_wall(drone, stop_distance):
        loop = True
        left = drone.getDistanceSensorData('DistanceLeft').distance
        straight = drone.getDistanceSensorData('DistanceStraight').distance
        while loop:
            moveByBodyVelocityAndZ(drone, straight*0.25, 0, -4, duration = 0.5)
            left = drone.getDistanceSensorData('DistanceLeft').distance
            if left > 20:
                loop = False
        time.sleep(2)
    def fly_to_right_wall(drone, stop_distance):
        loop = True
        straight = drone.getDistanceSensorData('DistanceStraight').distance
        while loop:
            moveByBodyVelocityAndZ(drone, straight*0.25, 0, -4, duration = 0.5)
            left = drone.getDistanceSensorData('DistanceLeft').distance
            right = drone.getDistanceSensorData('DistanceRight').distance
            if right > 15:
                loop = False
        time.sleep(2)

    fly_to_wall(drone, width)
    turn_right(drone, 90)
    fly_to_wall(drone, width)
    turn_left(drone, 90)
    moveToPosition(drone, Vector3r(140, -135, -4))
    turn_left(drone, 90)
    fly_to_wall(drone, width)
    turn_left(drone, 90)
    moveToPosition(drone, Vector3r(80, -120, -4))
    turn_right(drone, 90)
    fly_to_wall(drone, width)
    turn_left(drone, 90)
    fly_to_wall(drone, width)
    turn_right(drone, 90)



def land_on_target(drone:air_sim.MultirotorClient, **kwargs):
    def fly_to_wall(drone, stop_distance):
        loop = True
        straight = drone.getDistanceSensorData('DistanceStraight').distance
        while loop:
            moveByBodyVelocityAndZ(drone, straight*0.25, 0, -4, duration = 0.5)
            straight = drone.getDistanceSensorData('DistanceStraight').distance
            if straight <= stop_distance:
                loop = False
        time.sleep(2)
    straight = drone.getDistanceSensorData('DistanceStraight').distance
    moveToPosition(drone, Vector3r(-40, -85, -4))
    land(drone)
    



if __name__=="__main__":
    drone = init_sim()
    target_setup(drone, seed=5)
    takeoff(drone)


    
    # Start Timer
    start_time = time.perf_counter()

    hoop_race(drone)
    print("Hoop Race Time: ", time.perf_counter() - start_time)
    
    #set_pose(drone, 92, 0, 0) # Start False Wall
    false_wall(drone)
    print("False Wall Time: ", time.perf_counter() - start_time)
    
    #set_pose(drone, 107, -75, 0, yaw=-3.14/2) # Start Maze
    distance_maze(drone)
    print("Distance Maze Time: ", time.perf_counter() - start_time)
    
    #set_pose(drone, 50, -97, 0, yaw=3.14) # Start Search
    land_on_target(drone)
    print("Target Time: ", time.perf_counter() - start_time)

    # End Timer
    end_time = time.perf_counter()
    duration = end_time - start_time
    print(f'Total Time: {end_time - start_time}')

