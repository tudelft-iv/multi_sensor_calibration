input_urdf=$(rospack find prius_description)/urdf/prius.urdf
output_urdf=$(rospack find prius_description)/urdf/prius.urdf
radar_yaml="radar1.yaml"
camera_yaml="left.yaml"

# Update URDF with new radar pose
rosrun urdf_calibration urdf_calibration_cli $input_urdf optimization/results/$radar_yaml $output_urdf front_center_right_sonar_link front_center_right_sonar_joint
# Update URDF with new camera pose
rosrun urdf_calibration urdf_calibration_cli $output_urdf optimization/results/$camera_yaml $output_urdf left left_joint