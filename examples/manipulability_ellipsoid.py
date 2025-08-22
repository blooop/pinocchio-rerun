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
                                     app_id="ManipulabilityVisualizer", 
                                     rec_id="manipulability_demo")
rr.loadViewerModel()

# Get end-effector frame
frame_id = model.getFrameId("tool0")

# Create trajectory
T = 50
dt = 0.05
qs = np.zeros((T + 1, model.nq))

# Generate sinusoidal joint motion
for t in range(T + 1):
    time = t * dt
    for i in range(model.nq):
        qs[t, i] = 0.5 * np.sin(0.5 * time + i * np.pi / model.nq)

# Display robot and manipulability ellipsoid for each configuration
for t in range(T + 1):
    q = qs[t]
    
    # Display robot
    rr.display(q)
    
    # Draw manipulability ellipsoid
    rr.drawManipulabilityEllipsoid(frame_id, q, scale=0.1)

print("Manipulability ellipsoid visualization complete!")
print("Open Rerun viewer to see the animation.")