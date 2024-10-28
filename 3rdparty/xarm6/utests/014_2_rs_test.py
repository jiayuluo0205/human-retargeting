
from xarm6_interface.utils.realsense import build_rs_cam, Realsense


if __name__ == "__main__":
    

    
    import pyrealsense2 as rs
    import numpy as np
    import cv2

    # Create a context object. This object manages the handles to all connected RealSense devices
    context = rs.context()

    # List all the connected devices
    connected_devices = context.query_devices()
    if len(connected_devices) < 2:
        print("At least two RealSense cameras need to be connected.")
        exit(1)

    # Retrieve the serial numbers of the first two cameras
    serial_1 = connected_devices[0].get_info(rs.camera_info.serial_number)
    serial_2 = connected_devices[1].get_info(rs.camera_info.serial_number)

    # Print the serial numbers for your reference
    print(f"Camera 1 Serial Number: {serial_1}")
    print(f"Camera 2 Serial Number: {serial_2}")
    # Camera 1 Serial Number: 241122074374
    # Camera 2 Serial Number: 147122075879
    
    
    # Create pipelines for both cameras
    pipeline_1 = rs.pipeline()
    pipeline_2 = rs.pipeline()

    # Configure the first camera
    config_1 = rs.config()
    config_1.enable_device(serial_1)
    config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Configure color stream

    # Configure the second camera
    config_2 = rs.config()
    config_2.enable_device(serial_2)
    config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Configure color stream

    # Start the pipelines
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)

    try:
        while True:
            # Wait for a frame from each camera
            frames_1 = pipeline_1.wait_for_frames()
            frames_2 = pipeline_2.wait_for_frames()

            # Get color frames
            color_frame_1 = frames_1.get_color_frame()
            color_frame_2 = frames_2.get_color_frame()

            # Convert images to numpy arrays
            color_image_1 = np.asanyarray(color_frame_1.get_data())
            color_image_2 = np.asanyarray(color_frame_2.get_data())

            # Display images from both cameras
            cv2.imshow(f"Camera {serial_1} View", color_image_1)
            cv2.imshow(f"Camera {serial_2} View", color_image_2)

            # Exit the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop the pipelines
        pipeline_1.stop()
        pipeline_2.stop()
        cv2.destroyAllWindows()
