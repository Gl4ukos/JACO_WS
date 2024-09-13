import cv2
import numpy as np

# Define the chessboard size
chessboard_size = (9, 7)

# Define arrays to store object points and image points
obj_points = []
img_points = []

# Prepare the object points (0,0,0), (1,0,0), (2,0,0), ....,(6,6,0)
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Capture images for calibration
images = ['calibration_images/check_board_Color.png']

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if ret:
        img_points.append(corners)
        obj_points.append(objp)
        
        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Perform camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

# Save the camera calibration result
np.savez('camera_calib.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)