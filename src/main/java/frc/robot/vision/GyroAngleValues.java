package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * A record that represents the gyro's angle which is passed into the limelight's network tables, to the robot_orientation_set entry. Its values
 * were found in <a href=https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java>limelightlib sourcecode</a>. The
 * limelight is using it to calculate the MegaTag2 position.Rotation2d shall be used in degrees (and for rates, degrees per second). Everything
 * except the yaw is unnecessary.
 */
public record GyroAngleValues(Rotation2d yaw, double yawRate, Rotation2d pitch, double pitchRate, Rotation2d roll, double rollRate) {

	public GyroAngleValues() {
		this(Rotation3d.kZero);
	}

	public GyroAngleValues(Rotation3d angle) {
		this(angle, 0, 0, 0);
	}

	public GyroAngleValues(Rotation3d angle, double yawRate, double pitchRate, double rollRate) {
		this(
			Rotation2d.fromRadians(angle.getZ()),
			yawRate,
			Rotation2d.fromRadians(angle.getX()),
			pitchRate,
			Rotation2d.fromRadians(angle.getZ()),
			rollRate
		);
	}

	public double[] asArray() {
		return new double[] {
			this.yaw().getDegrees(),
			this.yawRate(),
			this.pitch().getDegrees(),
			this.pitchRate(),
			this.roll().getDegrees(),
			this.rollRate()};
	}

}
