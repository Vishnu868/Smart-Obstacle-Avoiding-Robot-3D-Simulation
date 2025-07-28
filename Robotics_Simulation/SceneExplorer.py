import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def explore_scene():
    """Explore the CoppeliaSim scene structure to find object paths"""
    try:
        # Connect to CoppeliaSim
        print("Connecting to CoppeliaSim...")
        client = RemoteAPIClient()
        sim = client.require('sim')
        
        # Stop simulation if running
        if sim.getSimulationState() != sim.simulation_stopped:
            sim.stopSimulation()
            time.sleep(1)
        
        print("\n--- SCENE STRUCTURE EXPLORATION ---")
        
        # Get all objects in the scene
        all_objects = []
        try:
            # Try to get the root object handle (scene root)
            root = sim.getObject("/")
            print(f"Root handle: {root}")
            
            # Get all objects in the scene
            all_objects = sim.getObjects(root, sim.objectany)
            print(f"Total objects found: {len(all_objects)}")
        except Exception as e:
            print(f"Error getting all objects: {e}")
            print("Trying alternative method...")
            
            # If the above fails, try an alternative approach
            try:
                # Try to get objects by type
                shapes = sim.getObjects(-1, sim.objectshape)
                joints = sim.getObjects(-1, sim.objectjoint)
                sensors = sim.getObjects(-1, sim.objectsensor)
                
                all_objects = shapes + joints + sensors
                print(f"Found {len(shapes)} shapes, {len(joints)} joints, {len(sensors)} sensors")
                print(f"Total objects found: {len(all_objects)}")
            except Exception as e2:
                print(f"Alternative method failed: {e2}")
        
        # Print information about each object
        print("\n--- OBJECT DETAILS ---")
        for i, obj in enumerate(all_objects):
            try:
                name = sim.getObjectAlias(obj)
                path = sim.getObjectAlias(obj, 1)  # Try to get the path
                
                # Get object type
                type_str = "Unknown"
                try:
                    type_int = sim.getObjectType(obj)
                    if type_int == sim.objectshape:
                        type_str = "Shape"
                    elif type_int == sim.objectjoint:
                        type_str = "Joint"
                    elif type_int == sim.objectdummy:
                        type_str = "Dummy"
                    elif type_int == sim.objectproximitysensor:
                        type_str = "Proximity Sensor"
                    elif type_int == sim.objectvisionsensor:
                        type_str = "Vision Sensor"
                except:
                    pass
                
                print(f"{i+1}. Name: {name}, Type: {type_str}, Path: {path}")
            except Exception as e:
                print(f"{i+1}. Error getting object details: {e}")
        
        # Try to list all objects by direct name
        print("\n--- DIRECT OBJECT SEARCH ---")
        print("Trying to list all objects in the scene...")
        
        # Potential wheel/motor names
        wheel_names = [
            "Revolute_joint_left_back",
            "Revolute_joint_right_back",
            "Revolute_joint_left_front",
            "Revolute_joint_right_front",
            "leftMotor",
            "rightMotor",
            "leftWheel",
            "rightWheel",
            "wheelLeft",
            "wheelRight"
        ]
        
        print("\nSearching for wheel joints...")
        for name in wheel_names:
            try:
                # Try different path formats
                paths_to_try = [
                    f"{name}",
                    f"./{name}",
                    f"/{name}",
                    f"/Scripts/{name}",
                    f"Scripts/{name}"
                ]
                
                for path in paths_to_try:
                    try:
                        handle = sim.getObject(path)
                        print(f"Found wheel: {path} (Handle: {handle})")
                        break
                    except:
                        continue
            except Exception as e:
                pass
        
        # Search for robot base
        robot_names = [
            "robot",
            "robot_base",
            "base",
            "robotBase",
            "chassis",
            "body"
        ]
        
        print("\nSearching for robot base...")
        for name in robot_names:
            try:
                paths_to_try = [
                    f"{name}",
                    f"./{name}",
                    f"/{name}",
                    f"/Scripts/{name}",
                    f"Scripts/{name}"
                ]
                
                for path in paths_to_try:
                    try:
                        handle = sim.getObject(path)
                        print(f"Found robot base: {path} (Handle: {handle})")
                        break
                    except:
                        continue
            except Exception as e:
                pass
        
        # Search for sensors
        sensor_names = [
            "Vision_sensor",
            "Proximity_sensor",
            "Proximity_sensor_0",
            "Proximity_sensor_1",
            "Proximity_sensor_2",
            "Proximity_sensor_Front",
            "Proximity_sensor_Left",
            "Proximity_sensor_Right",
            "visionSensor",
            "proximitySensor"
        ]
        
        print("\nSearching for sensors...")
        for name in sensor_names:
            try:
                paths_to_try = [
                    f"{name}",
                    f"./{name}",
                    f"/{name}",
                    f"/Scripts/{name}",
                    f"Scripts/{name}"
                ]
                
                for path in paths_to_try:
                    try:
                        handle = sim.getObject(path)
                        print(f"Found sensor: {path} (Handle: {handle})")
                        break
                    except:
                        continue
            except Exception as e:
                pass
        
        # Search for target
        target_names = [
            "target_cone",
            "target",
            "cone",
            "destination",
            "goal"
        ]
        
        print("\nSearching for target...")
        for name in target_names:
            try:
                paths_to_try = [
                    f"{name}",
                    f"./{name}",
                    f"/{name}",
                    f"/Scripts/{name}",
                    f"Scripts/{name}"
                ]
                
                for path in paths_to_try:
                    try:
                        handle = sim.getObject(path)
                        print(f"Found target: {path} (Handle: {handle})")
                        break
                    except:
                        continue
            except Exception as e:
                pass
        
        print("\n--- SCENE EXPLORATION COMPLETE ---")
        print("Please use the paths above to update your robot control code.")
        
    except Exception as e:
        print(f"Error during scene exploration: {e}")

# Main execution
if __name__ == "__main__":
    print("CoppeliaSim Scene Explorer")
    print("This script will explore your scene and help identify the correct object paths")
    print("===================================================================")
    
    # Explore the scene to find object paths
    explore_scene()
    
    print("\nExploration complete. Please update your main robot code with the correct paths.")