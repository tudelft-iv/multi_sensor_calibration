## Update a transform in the URDF with a known calibration

Run the example with:

	rosrun urdf_calibration urdf_calibration `rospack find urdf_calibration`/test/vehicle.urdf `rospack find urdf_calibration`/test/calibration.urdf `rospack find urdf_calibration`/test/output.urdf laser_mount_link laser_mount_joint

Be aware that this package is under development, highly experimental and will be completely refactored.
