#!/usr/bin/env python3
"""
Trajectory Executor for Dora-MoveIt + MuJoCo
============================================

Executes planned trajectories by sending joint commands to MuJoCo.
Interpolates between waypoints for smooth motion.
"""

import json
import time
import numpy as np
import pyarrow as pa
from typing import List, Optional
from dora import Node


class TrajectoryExecutor:
    """Executes motion trajectories on the robot"""
    
    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints
        self.trajectory: List[np.ndarray] = []
        self.current_waypoint_idx = 0
        self.current_joints: Optional[np.ndarray] = None
        self.prev_waypoint: Optional[np.ndarray] = None
        self.interpolation_progress = 0.0
        self.interpolation_speed = 0.1  # Progress per tick
        self.is_executing = False
        self.execution_count = 0
        self.current_trajectory_hash: Optional[int] = None
        self.last_command: Optional[np.ndarray] = None  # Remember last position
        
    def set_trajectory(self, trajectory: List[np.ndarray], trajectory_hash: int):
        """Set a new trajectory to execute"""
        # Ignore duplicate trajectories
        if self.current_trajectory_hash == trajectory_hash:
            return

        self.trajectory = trajectory
        self.current_trajectory_hash = trajectory_hash
        self.interpolation_progress = 0.0
        self.is_executing = True
        self.execution_count += 1

        if len(trajectory) > 0:
            # Start from first waypoint, execute from second
            self.prev_waypoint = trajectory[0]
            self.current_waypoint_idx = 1 if len(trajectory) > 1 else 0
            self.last_command = trajectory[0].copy()  # Update last_command to start position
            print(f"[Executor] New trajectory with {len(trajectory)} waypoints")
        
    def update_current_joints(self, joints: np.ndarray):
        """Update current joint positions from robot (only first 7 arm joints)"""
        # MuJoCo sends 9 values (7 arm + 2 gripper), we only need the 7 arm joints
        self.current_joints = joints[:self.num_joints].copy()
        
    def step(self) -> Optional[np.ndarray]:
        """
        Execute one step of trajectory.
        Returns joint command or None if not executing.
        """
        if not self.is_executing or len(self.trajectory) == 0:
            return self.last_command  # Keep holding last position

        if self.prev_waypoint is None:
            return self.last_command

        # Get current target waypoint
        target = self.trajectory[self.current_waypoint_idx]

        # Interpolate towards target
        self.interpolation_progress += self.interpolation_speed

        if self.interpolation_progress >= 1.0:
            # Reached waypoint, move to next
            self.prev_waypoint = target
            self.current_waypoint_idx += 1
            self.interpolation_progress = 0.0

            if self.current_waypoint_idx >= len(self.trajectory):
                # Trajectory complete
                self.is_executing = False
                self.last_command = self.trajectory[-1].copy()
                print(f"[Executor] Trajectory #{self.execution_count} complete!")
                return self.last_command

            target = self.trajectory[self.current_waypoint_idx]

        # Interpolate between previous waypoint and target
        t = min(self.interpolation_progress, 1.0)
        command = self.prev_waypoint + t * (target - self.prev_waypoint)
        self.last_command = command.copy()

        return command
    
    def get_status(self) -> dict:
        """Get execution status"""
        return {
            "is_executing": self.is_executing,
            "execution_count": self.execution_count,
            "current_waypoint": self.current_waypoint_idx,
            "total_waypoints": len(self.trajectory),
            "progress": self.interpolation_progress
        }


def main():
    print("=== Dora-MoveIt Trajectory Executor ===")

    node = Node()
    executor = TrajectoryExecutor(num_joints=7)

    # Initialize with safe config
    from robot_config import GEN72Config
    executor.current_joints = GEN72Config.SAFE_CONFIG.copy()
    executor.last_command = GEN72Config.SAFE_CONFIG.copy()  # Initialize last_command
    print(f"Initialized with safe config: {executor.current_joints[:6]}...")

    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            input_id = event["id"]
            
            if input_id == "trajectory":
                # Receive new trajectory from planner
                try:
                    traj_flat = event["value"].to_numpy()
                    metadata = event.get("metadata", {})
                    num_waypoints = metadata.get("num_waypoints", len(traj_flat) // 7)
                    num_joints = metadata.get("num_joints", 7)

                    trajectory = traj_flat.reshape(num_waypoints, num_joints)
                    trajectory_list = [trajectory[i] for i in range(num_waypoints)]

                    # Insert current position as first waypoint for smooth transition
                    # Use last_command if available (more accurate than current_joints)
                    if executor.last_command is not None:
                        trajectory_list.insert(0, executor.last_command.copy())
                    elif executor.current_joints is not None:
                        trajectory_list.insert(0, executor.current_joints.copy())

                    # Compute hash to detect duplicate trajectories
                    traj_hash = hash(traj_flat.tobytes())
                    executor.set_trajectory(trajectory_list, traj_hash)

                except Exception as e:
                    print(f"[Executor] Trajectory error: {e}")
                    
            elif input_id == "joint_positions":
                # Update current robot state
                try:
                    joints = event["value"].to_numpy()
                    executor.update_current_joints(joints)
                except Exception as e:
                    pass
                    
            elif input_id == "tick":
                # Execute trajectory step
                command = executor.step()

                # Always send command (current position if not executing)
                if command is not None:
                    node.send_output(
                        "joint_commands",
                        pa.array(command, type=pa.float32())
                    )
                elif executor.current_joints is not None:
                    # Send current position when idle
                    node.send_output(
                        "joint_commands",
                        pa.array(executor.current_joints, type=pa.float32())
                    )
                
                # Send status periodically
                status = executor.get_status()
                status_bytes = json.dumps(status).encode('utf-8')
                node.send_output(
                    "execution_status",
                    pa.array(list(status_bytes), type=pa.uint8())
                )
                    
        elif event_type == "STOP":
            print("Trajectory executor stopping...")
            break


if __name__ == "__main__":
    main()

