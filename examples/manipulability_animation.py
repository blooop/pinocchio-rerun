import pinocchio_rerun
import example_robot_data as erd
import pinocchio as pin
import numpy as np

# Load robot model
robot = erd.load("ur5")
model = robot.model
visual_model = robot.visual_model

# Create visualizer
rr = pinocchio_rerun.RerunVisualizer(model, visual_model, 
                                     app_id="ManipulabilityAnimation", 
                                     rec_id="manipulability_animation")
rr.loadViewerModel()

# Get end-effector frame
frame_id = model.getFrameId("tool0")


# Define key poses for the robot arm to move through
key_poses = [
    np.array([0.0, -1.2, 1.5, -1.8, -1.5, 0.0]),     # Home position
    np.array([0.8, -0.8, 1.2, -2.0, -1.5, 0.0]),     # Right reach
    np.array([0.8, -0.3, 0.8, -1.2, -1.5, 0.0]),     # Right high
    np.array([0.0, -0.3, 0.8, -1.2, -1.5, 0.0]),     # Center high
    np.array([-0.8, -0.3, 0.8, -1.2, -1.5, 0.0]),    # Left high
    np.array([-0.8, -0.8, 1.2, -2.0, -1.5, 0.0]),    # Left reach
    np.array([-0.8, -1.5, 2.0, -2.2, -1.5, 0.0]),    # Left low
    np.array([0.0, -1.5, 2.0, -2.2, -1.5, 0.0]),     # Center low
    np.array([0.8, -1.5, 2.0, -2.2, -1.5, 0.0]),     # Right low
    np.array([0.0, -1.2, 1.5, -1.8, -1.5, 0.0]),     # Back to home
]

print(f"Interpolating between {len(key_poses)} key poses")

# Interpolate between key poses for smoother animation
def interpolate_trajectory(poses, steps_per_segment=30):
    trajectory = []
    
    for i in range(len(poses) - 1):
        start_pose = poses[i]
        end_pose = poses[i + 1]
        
        # Create interpolation points between current and next pose
        for j in range(steps_per_segment):
            alpha = j / (steps_per_segment - 1) if steps_per_segment > 1 else 0
            interpolated_q = (1 - alpha) * start_pose + alpha * end_pose
            trajectory.append(interpolated_q.copy())
    
    return trajectory

# Generate smooth trajectory with more interpolation steps for smoothness
trajectory = interpolate_trajectory(key_poses, steps_per_segment=30)
total_duration = 25.0  # Total animation duration in seconds
dt = total_duration / len(trajectory)

print(f"Generated smooth trajectory with {len(trajectory)} points over {total_duration}s")

# Animate robot and ellipsoid transforms only
print("Recording robot and ellipsoid animation...")

for step, q in enumerate(trajectory):
    current_time = step * dt
    
    # Set timeline for this frame
    rr.setTimeSeconds("animation", current_time)
    
    # Update forward kinematics and display robot
    pin.forwardKinematics(model, rr.data, q)
    rr.display(q)
    rr.drawManipulabilityEllipsoid(frame_id, q, scale=0.08, static_log=False)
    
    if step % 50 == 0:  # Print progress
        print(f"Frame {step}/{len(trajectory)} at t={current_time:.2f}s")

print("\nAnimation complete!")
print(f"- Robot animation: {len(trajectory)} poses over {total_duration:.1f} seconds")
print("- Manipulability ellipsoid animated at every trajectory step")
print("- Use timeline scrubber to see how manipulability changes in real-time")
print("- The ellipsoid shows velocity manipulability - size and shape indicate capability")