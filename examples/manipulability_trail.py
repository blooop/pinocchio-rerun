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
                                     app_id="ManipulabilityTrail", 
                                     rec_id="manipulability_trail")
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

# Interpolate between key poses
def interpolate_trajectory(poses, steps_per_segment=15):
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

# Generate smooth trajectory
trajectory = interpolate_trajectory(key_poses, steps_per_segment=20)
total_duration = 30.0  # Total animation duration in seconds
dt = total_duration / len(trajectory)

print(f"Generated smooth trajectory with {len(trajectory)} points over {total_duration}s")

# Create static ellipsoid trail that persists across all timeline positions
ellipsoid_counter = 0
trail_frequency = 8  # Leave ellipsoid every N steps for a nice trail

print("Creating persistent ellipsoid trail...")
for step, q in enumerate(trajectory):
    if step % trail_frequency == 0:
        # Create static ellipsoid that persists regardless of timeline position
        rr.drawManipulabilityEllipsoid(frame_id, q, scale=0.06, static_log=True)
        ellipsoid_counter += 1
        print(f"Created static ellipsoid {ellipsoid_counter} at step {step}")

# Now create the robot animation
print("Animating robot through trajectory...")
rr.play(trajectory, dt, "robot_animation")

print(f"\nAnimation complete!")
print(f"- Robot animation: {len(trajectory)} poses over {total_duration:.1f} seconds")
print(f"- Trail: {ellipsoid_counter} ellipsoids placed at key positions")

print(f"\nRecording complete! Created {ellipsoid_counter} manipulability ellipsoids.")
print(f"Total duration: {total_duration:.1f} seconds")
print("\nIn the Rerun viewer, you can:")
print("- Use the timeline scrubber to navigate through the recorded motion")
print("- See how manipulability changes as the arm moves through its workspace") 
print("- Pause at any point to examine the robot configuration and ellipsoid shape")
print("- The ellipsoids show velocity manipulability at different workspace positions")