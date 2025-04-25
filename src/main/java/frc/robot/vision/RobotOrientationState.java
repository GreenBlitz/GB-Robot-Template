package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * A record that represents the robot's angles which is passed into the limelight's network tables, to the robot_orientation_set entry. Its
 * values were found in <a href=https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java>limelightlib
 * sourcecode</a>. The limelight is using it to calculate the MegaTag2 position.Rotation2d shall be used in degrees (and for rates, degrees per
 * second). Everything except the yaw is unnecessary.
 */
public record RobotOrientationState(
	Rotation2d yaw,
	Rotation2d yawVelocity,
	Rotation2d pitch,
	Rotation2d pitchRate,
	Rotation2d roll,
	Rotation2d rollRate
) {

	public RobotOrientationState() {
		this(Rotation3d.kZero);
	}

	public RobotOrientationState(Rotation3d angle) {
		this(angle, Rotation3d.kZero);
	}

	public RobotOrientationState(Rotation3d angle, Rotation3d angularVelocity) {
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
			this.pitchRate().getDegrees(),
			this.roll().getDegrees(),
			this.rollRate().getDegrees()};
	}

}
