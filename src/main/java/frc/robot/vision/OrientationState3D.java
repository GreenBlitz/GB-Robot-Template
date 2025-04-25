package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * A record that represents the robot's angles which is passed into the limelight's network tables, to the robot_orientation_set entry. Its
 * values were found in <a href=https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java>limelightlib
 * sourcecode</a>. The limelight is using it to calculate the MegaTag2 position.Rotation2d shall be used in degrees (and for rates, degrees per
 * second). Everything except the yaw is unnecessary.
 */
public record OrientationState3D(
	Rotation2d yaw,
	Rotation2d yawVelocity,
	Rotation2d pitch,
	Rotation2d pitchVelocity,
	Rotation2d roll,
	Rotation2d rollVelocity
) {

	public OrientationState3D() {
		this(Rotation3d.kZero);
	}

	public OrientationState3D(Rotation3d angle) {
		this(angle, Rotation3d.kZero);
	}

	public OrientationState3D(Rotation3d angle, Rotation3d angularVelocity) {
		this(
			Rotation2d.fromRadians(angle.getZ()),
			Rotation2d.fromRadians(angularVelocity.getZ()),
			Rotation2d.fromRadians(angle.getY()),
			Rotation2d.fromRadians(angularVelocity.getY()),
			Rotation2d.fromRadians(angle.getX()),
			Rotation2d.fromRadians(angularVelocity.getX())
		);
	}

	public double[] asArray() {
		return new double[] {
			this.yaw().getDegrees(),
			this.yawVelocity().getDegrees(),
			this.pitch().getDegrees(),
			this.pitchVelocity().getDegrees(),
			this.roll().getDegrees(),
			this.rollVelocity().getDegrees()};
	}

}
