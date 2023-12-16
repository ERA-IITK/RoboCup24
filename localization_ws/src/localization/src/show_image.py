import cv2
import os

# Folder path containing the calibration images
folder_path = '/home/suryansh/Documents/GitHub/RoboCup24/localization/calibration_images/'

# Get a list of image file names in the folder
image_files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]

# Loop through each image file and display it
for image_file in image_files:
    # Read the image
    image = cv2.imread(os.path.join(folder_path, image_file))

    if image is not None:
        # Display the image
        cv2.imshow(image_file, image)
        cv2.waitKey(1000)  # Display each image for 1 second (adjust as needed)

# Close all OpenCV windows when done
cv2.destroyAllWindows()
