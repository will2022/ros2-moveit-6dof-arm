#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

def main():
    rclpy.init()
    
    # Instantiate MoveItPy
    my_robot = MoveItPy(node_name="moveit_py_example")
    
    # Get the planning component for the manipulator group
    manipulator = my_robot.get_planning_component("manipulator")
    
    # 1. Plan to a joint target
    print("\n----------------------------------------------------------")
    print("Planning to joint target")
    print("----------------------------------------------------------")
    
    # Set joint target
    manipulator.set_start_state_to_current_state()
    manipulator.set_goal_state(configuration_name="ready")
    
    # Plan
    plan_result = manipulator.plan()
    
    if plan_result:
        print("Planning successful!")
        # Execute
        print("Executing...")
        manipulator.execute(plan_result, blocking=True)
        print("Execution complete!")
    else:
        print("Planning failed!")

    # 2. Plan to "home" configuration
    print("\n----------------------------------------------------------")
    print("Planning to 'home' configuration")
    print("----------------------------------------------------------")
    
    manipulator.set_start_state_to_current_state()
    
    # Plan to "home" named state as an example of a different target
    manipulator.set_goal_state(configuration_name="home")
    
    plan_result = manipulator.plan()
    
    if plan_result:
        print("Planning successful!")
        print("Executing...")
        manipulator.execute(plan_result, blocking=True)
        print("Execution complete!")
    else:
        print("Planning failed!")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
