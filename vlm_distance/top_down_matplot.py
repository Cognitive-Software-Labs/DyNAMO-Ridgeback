import matplotlib.pyplot as plt
import math

camera_x, camera_z = 0.0, 0.0
robot_x, robot_z = 0.26, 0.92

distance = math.sqrt((robot_x - camera_x)**2 + (robot_z - camera_z)**2)

plt.figure(figsize=(6, 6))

plt.scatter(camera_x, camera_z, s=120, label="Ridgeback / Camera")
plt.scatter(robot_x, robot_z, s=120, label="Robot")

plt.plot([camera_x, robot_x], [camera_z, robot_z], "--")

plt.text(camera_x + 0.02, camera_z + 0.02, "Camera (0, 0)")
plt.text(robot_x + 0.02, robot_z + 0.02, f"Robot ({robot_x}, {robot_z})")

mid_x = (camera_x + robot_x) / 2
mid_z = (camera_z + robot_z) / 2
plt.text(mid_x + 0.03, mid_z, f"Distance = {distance:.2f} m")

plt.xlabel("x (left / right)")
plt.ylabel("z (forward / back)")
plt.title("Top-Down Relative Position")
plt.xlim(0, 0.7)
plt.ylim(0, 1.1)
plt.grid(True)
plt.legend()
plt.axis("equal")

plt.show()