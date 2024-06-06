#!/usr/bin/env python3

import pyrealsense2 as rs
import cv2
import numpy as np
import time

depth_frame = None
depth_image = None
color_image = None
def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN and depth_frame:
        print('x: ', x, 'y: ', y)
        dist = depth_frame.get_distance(y, x)
        print('depth (m): ', dist)
        # convert to 3d coordinate
        y_3d, x_3d, _ = rs.rs2_deproject_pixel_to_point(camera_intr, [y, x], dist)
        print('x_3d: ', x_3d, 'y_3d: ', y_3d)
        output_dict = {'handle_3d': np.array([x_3d, y_3d, dist]),
                          'classification': 'Left-hinged',
                          'axis': 'NA',
                          'radius': 'NA'}
        np.save("maskrcnn_cls_kpt_fulldataset/maskrcnn_prediction_output.npy", output_dict)
        cv2.imwrite("example_depth_image.jpg", depth_image)
        cv2.imwrite("example_color_image.jpg", color_image)
    
try: 
    # Note: width/heights are swapped since the images will be rotated 90 degrees from what the camera captures.
    resolution_depth = [640, 480]
    resolution_color = [640, 480]   
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_device('153122077062') # D435
    config.enable_device('130322271205') # D405

    config.enable_stream(rs.stream.depth, resolution_depth[0], resolution_depth[1], rs.format.z16, 30)
    config.enable_stream(rs.stream.color, resolution_color[0], resolution_color[1], rs.format.bgr8, 30)
    # start streaming
    rs_cfg = pipeline.start(config)
    # get camera instrinsics
    profile = rs_cfg.get_stream(rs.stream.color)
    camera_intr = profile.as_video_stream_profile().get_intrinsics()
    print(rs_cfg.get_device().first_depth_sensor().get_depth_scale())
    # for depth to color alignment
    align_to = rs.stream.color
    align = rs.align(align_to)
    print(camera_intr)
    first_frame = True
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame() 
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame: 
            continue
            
        # convert frame to image
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=-0.04, beta=255.0), cv2.COLORMAP_OCEAN)
        color_image = np.asanyarray(color_frame.get_data())
        # Rotate and flip images
        depth_image = np.moveaxis(depth_image, 0, 1)
        # depth_image = np.fliplr(depth_image)
        color_image = np.moveaxis(color_image, 0, 1)
        # color_image = np.fliplr(color_image)
        
        cv2.imshow('Depth', depth_image)
        cv2.imshow('Color', color_image)
        
        if first_frame:  
            # setup mouse click event
            cv2.setMouseCallback('Color', click_event)
            first_frame = False
            
        if cv2.waitKey(1) == ord('q'):
            break

except KeyboardInterrupt:
     print("Stopping pipeline")
     
finally:
    pipeline.stop()
