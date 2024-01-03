import cv2
import numpy as np
import yaml

# Assuming the camera parameters are defined
camera_matrix_data = np.array([435.72155, 0., 342.73125, 0., 439.05066, 234.28296, 0., 0., 1.]).reshape(3, 3)
distortion_coefficients_data = np.array([-0.327652, 0.079815, -0.000013, -0.001481, 0.000000])
rectification_matrix_data = np.eye(3)
projection_matrix_data = np.array([333.84367, 0., 356.34991, 0., 0., 389.19197, 232.73828, 0., 0., 0., 1., 0.]).reshape(3, 4)

image_size = (640, 480)  # Adjust according to your image size
cam_mtx = camera_matrix_data
dis_cef = distortion_coefficients_data
R = rectification_matrix_data
cam_mtx_ud = projection_matrix_data[:, :3]

# Create map_x and map_y matrices
map_x = np.zeros(image_size, dtype=np.float32)
map_y = np.zeros(image_size, dtype=np.float32)

pts_distort = []
for y in range(image_size[1]):
    for x in range(image_size[0]):
        pts_distort.append((x, y))

pts_ud = cv2.undistortPoints(np.array(pts_distort).reshape(-1, 1, 2), cam_mtx, dis_cef, R, cam_mtx_ud)

for y in range(image_size[1]):
    for x in range(image_size[0]):
        pt = pts_ud[y * image_size[0] + x][0]
        map_x[y, x] = pt[0]
        map_y[y, x] = pt[1]

# Assuming 'image_ud' is defined
image_distort = cv2.remap(image_ud, map_x, map_y, interpolation=cv2.INTER_LINEAR)

