import time
import sys
import math
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def main():
    print("Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    # Make sure CoppeliaSim is running
    try:
        sim.getSimulationState()
        print("Successfully connected to CoppeliaSim")
    except Exception as e:
        print(f"ERROR: Could not connect to CoppeliaSim: {e}")
        return
    
    # Get object handles with better error reporting
    print("\nGetting object handles...")
    
    # Try different names for the robot base
    robot_base = -1
    base_names = ['Robotnik_Summit_XL', 'Summit_XL', 'SummitXL', 'Robotnik_Summit_XL_visible', 'Summit_XL_visible']
    for name in base_names:
        try:
            robot_base = sim.getObject(f'./{name}')
            print(f"Found robot base: {name}")
            break
        except Exception:
            pass
    
    if robot_base == -1:
        print("ERROR: Could not find robot base. Available objects:")
        print_all_objects(sim)
        return
    
    # Get wheel joint handles
    wheel_joints = []
    wheel_names = [
        'joint_front_left_wheel', 'joint_front_right_wheel', 
        'joint_back_left_wheel', 'joint_back_right_wheel',
        'joint_back_left', 'joint_front_left', 
        'joint_back_right', 'joint_front_right'
    ]
    
    for name in wheel_names:
        try:
            handle = sim.getObject(f'./{name}')
            wheel_joints.append(handle)
            print(f"Found wheel: {name}")
        except Exception:
            pass
    
    if not wheel_joints:
        print("ERROR: Could not find any wheel joints. Available objects:")
        print_all_objects(sim)
        return
    
    # Get target (Goal) handle
    goal = -1
    goal_names = ['Goal', 'target', 'destination', 'Target', 'Destination']
    for name in goal_names:
        try:
            goal = sim.getObject(f'./{name}')
            print(f"Found goal: {name}")
            break
        except Exception:
            pass
    
    if goal == -1:
        print("ERROR: Could not find goal object. Available objects:")
        print_all_objects(sim)
        return
    
    # Test setting wheel velocities
    print("\nTesting wheel movement...")
    sim.startSimulation()
    
    try:
        # Set all wheels to a small velocity to test
        for wheel in wheel_joints:
            sim.setJointTargetVelocity(wheel, 1.0)
        
        print("Set wheel velocities to 1.0")
        print("If the robot doesn't move, check the wheel joint configurations")
        
        # Wait a moment
        time.sleep(5)
        
        # Stop the wheels
        for wheel in wheel_joints:
            sim.setJointTargetVelocity(wheel, 0.0)
        
        print("Stopped wheels")
    except Exception as e:
        print(f"ERROR setting wheel velocities: {e}")
    
    sim.stopSimulation()
    print("Test complete")

def print_all_objects(sim):
    # Print all objects in the scene to help with debugging
    try:
        all_objects = sim.getObjects()
        print(f"Found {len(all_objects)} objects in the scene:")
        for i, obj in enumerate(all_objects):
            try:
                name = sim.getObjectName(obj)
                print(f"  {i+1}. {name} (handle: {obj})")
            except:
                print(f"  {i+1}. [Unnamed object] (handle: {obj})")
    except Exception as e:
        print(f"Error getting object list: {e}")

if __name__ == "__main__":
    main()