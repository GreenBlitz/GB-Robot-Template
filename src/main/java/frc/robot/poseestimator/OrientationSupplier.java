package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.Filter;

public interface OrientationSupplier {

	InputSignal<Rotation2d> getYawSignal();

	Rotation3d getLatestOrientation();

	Rotation3d[] getAllOrientations();

	Filter isDataReliable();

	void setYaw(Rotation2d yaw);

	void setOrientation(Rotation3d orientation);

	void updateOrientationSupplier();

}
