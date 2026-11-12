package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation3d;

public interface OrientationSupplier {

	Rotation3d getLatestOrientation();

	Rotation3d[] getAllOrientations();

}
