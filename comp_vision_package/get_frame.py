import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import time
import json

print(cv2.__version__)
calibration_data = '''
{
  "baseline": "-50.002",
  "intrinsic_left.x.x": "0.50337",
  "intrinsic_left.x.y": "0.804595",
  "intrinsic_left.x.z": "0.501416",
  "intrinsic_left.y.x": "0.502141",
  "intrinsic_left.y.y": "-0.0557814",
  "intrinsic_left.y.z": "0.0657176",
  "intrinsic_left.z.x": "-0.000719017",
  "intrinsic_left.z.y": "-0.000230586",
  "intrinsic_left.z.z": "-0.021544",
  "intrinsic_right.x.x": "0.499718",
  "intrinsic_right.x.y": "0.798685",
  "intrinsic_right.x.z": "0.50452",
  "intrinsic_right.y.x": "0.502384",
  "intrinsic_right.y.y": "-0.0559515",
  "intrinsic_right.y.z": "0.0690624",
  "intrinsic_right.z.x": "0.000225481",
  "intrinsic_right.z.y": "0.000544337",
  "intrinsic_right.z.z": "-0.0220516",
  "rectified.0.fx": "964.679",
  "rectified.0.fy": "964.679",
  "rectified.0.height": "1080",
  "rectified.0.ppx": "973.063",
  "rectified.0.ppy": "542.033",
  "rectified.0.width": "1920"
}
'''

calib = json.loads(calibration_data)

# Extract the intrinsic parameters for the left camera
fx = float(calib["rectified.0.fx"])
fy = float(calib["rectified.0.fy"])
cx = float(calib["rectified.0.ppx"])
cy = float(calib["rectified.0.ppy"])

# Create the camera matrix
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

# Assuming no distortion for simplicity
dist_coeffs = np.zeros((5, 1))


# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream color frames
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

#alligning colour to depth
align_to = rs.stream.color
align = rs.align(align_to)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
aruco_params = aruco.DetectorParameters()

time.sleep(2) # !!! The camera need a couple of seconds to balance its colours
try:
    while(True):
    
        # Wait for a coherent color frame
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            print("shit")
            continue

        # Convert image to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        #converting to grayscale
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        #gray = np.float32(gray)
        dst = cv2.cornerHarris(gray, 2, 3, 0.04)
        dst = cv2.dilate(dst, None)


        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        marker_length = 0.025
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        color_image[dst > 0.01 * dst.max()] = [0, 0, 255]

        if ids is not None:
            # Draw the markers
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

            # Draw circles on the corners
            for i in range(len(ids)):
                tvec = tvecs[i][0]
                distance_to_camera = np.linalg.norm(tvec)
                cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
                print(f'Distance from camera to marker {ids[i][0]}: {distance_to_camera:.2f} meters')

                marker_id = ids[i][0]
                print(f'Marker ID: {marker_id}')
                for corner in corners[i]:
                    for point in corner:
                        x, y = int(point[0]), int(point[1])
                        #print(f'Corner coordinates: ({x}, {y})')
                        cv2.circle(color_image, (x, y), 5, (0, 255, 0), -1)


                #calculating distance between two given aruco marker ids
                """
                id1, id2 = 1, 2  # Example marker IDs
                if id1 in ids and id2 in ids:
                    # Get the translation vectors of the two markers
                    tvec1 = tvecs[np.where(ids == id1)[0][0]][0]
                    tvec2 = tvecs[np.where(ids == id2)[0][0]][0]

                    # Compute Euclidean distance
                    distance = np.linalg.norm(tvec1 - tvec2)
                    print(f'Distance between marker {id1} and marker {id2}: {distance:.2f} meters')
                """


        cv2.imshow('Corners', color_image)


        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()