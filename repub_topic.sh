rosrun topic_tools throttle messages /camera/color/image_raw 1.0 /repub_color_raw &
rosrun topic_tools throttle messages /camera/depth/color/points 1.0 /repub_points &
rosrun topic_tools throttle messages /camera/depth/image_rect_raw 1.0 /repub_depth_rect &
rosrun topic_tools throttle messages /camera/infra/image_raw 1.0 /repub_infra_raw