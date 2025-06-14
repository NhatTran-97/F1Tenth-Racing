import cv2
import numpy as np
import pyrealsense2 as rs

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Enable only color stream at 640x640 resolution
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Create a video writer for saving the video in .mp4 format
color_writer = cv2.VideoWriter('color_video.mp4', cv2.VideoWriter_fourcc(*'H264'), 30, (640, 480))

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # Get the color frame
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Save the frame in .mp4 format
        color_writer.write(color_image)
        print("Writing frame to video...")  # Debugging print
    
        # Display the color image
        cv2.imshow('Color Image', color_image)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    color_writer.release()
    cv2.destroyAllWindows()
    print("Video writer released and file saved.")
