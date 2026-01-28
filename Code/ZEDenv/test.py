import cv2
import numpy as np
import pyzed.sl as sl

# Initialize the ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # You can change resolution
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE for faster depth
init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED
init_params.sdk_verbose = True

# Open the ZED camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"Failed to open ZED camera: {err}")
    exit()

# Create an image object for depth frame
depth_image = sl.Mat()

while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Retrieve depth image
        zed.retrieve_measure(depth_image, sl.MEASURE.DEPTH)

        # Convert depth image to numpy array
        depth_data = depth_image.get_data()

        # Normalize the depth data to visualize in grayscale
        depth_display = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = np.uint8(depth_display)  # Convert to 8-bit image

        # Optionally apply color map for better visualization
        depth_display = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

        # Display the depth map
        cv2.imshow("Real-Time Depth", depth_display)

    # Check for 'Esc' key to exit
    key = cv2.waitKey(1)
    if key == 27:
        break

# Clean up and close the camera
zed.close()
cv2.destroyAllWindows()
