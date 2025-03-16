package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * A class that represents the robot's angles which is passed into the limelight's network tables, to the robot_orientation_set entry. Its
 * values were found in <a href=https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java>limelightlib
 * sourcecode</a>. The limelight is using it to calculate the MegaTag2 position.Rotation2d shall be used in degrees (and for rates, degrees per
 * second). Everything except the yaw is unnecessary.
 */
public class RobotAngleValues {

	private Rotation2d yaw;

	private double yawRate;

	private Rotation2d pitch;

	private double pitchRate;

	private Rotation2d roll;

	private double rollRate;

	public RobotAngleValues(Rotation2d yaw, double yawRate, Rotation2d pitch, double pitchRate, Rotation2d roll, double rollRate) {
		this.yaw = yaw;
		this.yawRate = yawRate;
		this.pitch = pitch;
		this.pitchRate = pitchRate;
		this.roll = roll;
		this.rollRate = rollRate;
	}

	public RobotAngleValues() {
		this(Rotation3d.kZero);
	}

	public RobotAngleValues(Rotation3d angle) {
		this(angle, 0, 0, 0);
	}

	public RobotAngleValues(Rotation3d angle, double yawRate, double pitchRate, double rollRate) {
		this(
			Rotation2d.fromRadians(angle.getZ()),
			yawRate,
			Rotation2d.fromRadians(angle.getY()),
			pitchRate,
			Rotation2d.fromRadians(angle.getX()),
			rollRate
		);
	}

	public void setValues(Rotation2d yaw, double yawRate, Rotation2d pitch, double pitchRate, Rotation2d roll, double rollRate) {
		this.yaw = yaw;
		this.yawRate = yawRate;
		this.pitch = pitch;
		this.pitchRate = pitchRate;
		this.roll = roll;
		this.rollRate = rollRate;
	}

	public Rotation2d getYaw() {
		return yaw;
	}

	public void setYaw(Rotation2d yaw) {
		this.yaw = yaw;
	}

	public double getYawRate() {
		return yawRate;
	}

	public void setYawRate(double yawRate) {
		this.yawRate = yawRate;
	}

	public Rotation2d getPitch() {
		return pitch;
	}

	public void setPitch(Rotation2d pitch) {
		this.pitch = pitch;
	}

	public double getPitchRate() {
		return pitchRate;
	}

	public void setPitchRate(double pitchRate) {
		this.pitchRate = pitchRate;
	}

	public Rotation2d getRoll() {
		return roll;
	}

	public void setRoll(Rotation2d roll) {
		this.roll = roll;
	}

	public double getRollRate() {
		return rollRate;
	}

	public void setRollRate(double rollRate) {
		this.rollRate = rollRate;
	}

	public double[] asArray() {
		return new double[] {
			this.yaw.getDegrees(),
			this.yawRate,
			this.pitch.getDegrees(),
			this.pitchRate,
			this.roll.getDegrees(),
			this.rollRate
		};
	}

}
