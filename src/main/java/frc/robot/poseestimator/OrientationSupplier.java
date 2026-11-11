package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation3d;

import java.util.function.Supplier;

public interface OrientationSupplier {

	Rotation3d getLatestOrientation();

	Rotation3d[] getAllOrientations();

}
