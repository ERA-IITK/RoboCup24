import numpy as np
import cv2
import glob

# Define the checkerboard size (5x8 squares)
checkerboard_size = (4, 7)

# Define the size of a single square in millimeters
square_size = 30.0  # Adjust this based on your checkerboard

# Create arrays to store object points (3D) and image points (2D)
object_points = []  # Object points in the real world (3D)
image_points = []   # Image points in the image (2D)

# Load checkerboard images from a folder (replace with your folder path)
image_files = glob.glob('/home/suryansh/Documents/GitHub/RoboCup24/localization/calibration_images/*.jpeg')
if not image_files:
    print("Error: No image files found.")
else:
    # Define fisheye calibration flags
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND

    # Loop through each checkerboard image
    for image_file in image_files:
        image = cv2.imread(image_file)

        if image is None:
            print(f"Error: Unable to load image {image_file}")
            continue

        # Find chessboard corners in the image
        found, corners = cv2.findChessboardCorners(image, checkerboard_size)

        if found:
            # Add object points (3D coordinates of checkerboard) and image points (2D corners)
            obj = np.zeros((checkerboard_size[0] * checkerboard_size[1], 1, 3), np.float32)
            obj[:, :, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 1, 2) * square_size
            object_points.append(obj)
            image_points.append(corners)

            # Draw and display the chessboard corners (optional)
            cv2.drawChessboardCorners(image, checkerboard_size, corners, found)
            cv2.imshow("Chessboard Corners", image)
            cv2.waitKey(100)  # Adjust the delay as needed

    # Perform fisheye camera calibration for all images
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(object_points))]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(object_points))]
    calibration_flags |= cv2.fisheye.CALIB_FIX_SKEW
    ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        object_points,
        image_points,
        (image.shape[1], image.shape[0]),
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )

    # Print the intrinsic matrix (K) and distortion coefficients
    print("Intrinsic Matrix (K):\n", K)
    print("\nDistortion Coefficients:\n", D)

    # Save the calibration results
    cv2_file = cv2.FileStorage("calibration_results.yml", cv2.FILE_STORAGE_WRITE)
    cv2_file.write("cameraMatrix", K)
    cv2_file.write("distCoeffs", D)
    cv2_file.release()

    cv2.destroyAllWindows()
