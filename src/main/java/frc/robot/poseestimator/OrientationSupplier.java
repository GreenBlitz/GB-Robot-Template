package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation3d;

import java.util.Optional;
import java.util.function.Supplier;

public interface OrientationSupplier {

	Optional<Rotation3d> getLatestOrientation();

	Optional<Rotation3d[]> getAllOrientations();

}
