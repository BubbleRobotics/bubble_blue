import numpy as np

# Define geometry basis
bot_left = np.array([11.835702, 13.253233, -5])
next_bot_left = np.array([11.319732, 11.416660, -5])
bot_right = bot_left + (next_bot_left - bot_left) / 2
top_left = np.array([11.835702, 13.253233, -3.5])

horizontal_vec = bot_right - bot_left
vertical_vec = top_left - bot_left

normal_vec = np.cross(horizontal_vec, vertical_vec)
max_thickness = 0.25
normal_vec_norm = normal_vec / np.linalg.norm(normal_vec)
collision_box_size = np.array([
    np.linalg.norm(horizontal_vec)*1.1,
    max_thickness,
    np.linalg.norm(vertical_vec)])

collision_box_pose = bot_left + horizontal_vec / 2 * 1.1 + vertical_vec / 2 + (max_thickness / 2) * normal_vec_norm
print("Collision box size (x,y,z): ", collision_box_size)
print("Collision box pose (x,y,z): ", collision_box_pose)

goal_position = bot_left + normal_vec_norm * 0.5
print("Goal position (x,y,z): ", goal_position)