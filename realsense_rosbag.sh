

rosbag record -O realsense.bag \
/camera/color/camera_info \
/repub_color_raw \
/camera/depth/camera_info \
/repub_points \
/repub_depth_rect \
/camera/extrinsics/depth_to_color \
/camera/infra/camera_info \
/repub_infra_raw \
/camera/l500_depth_sensor/parameter_descriptions \
/camera/l500_depth_sensor/parameter_updates \
/camera/motion_module/parameter_descriptions \
/camera/motion_module/parameter_updates \
/camera/pointcloud/parameter_descriptions \
/camera/pointcloud/parameter_updates \
/camera/realsense2_camera_manager/bond \
/camera/rgb_camera/parameter_descriptions \
/camera/rgb_camera/parameter_updates \
/tf_static \
/tf
/joint_states

