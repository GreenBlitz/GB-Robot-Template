package frc.utils.pose;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import frc.utils.math.ToleranceMath;

public class PoseUtil {

	public static boolean isAccelerationHigh(Translation2d imuAccelerationG, double minimumIMUAccelerationG) {
		return imuAccelerationG.getNorm() >= minimumIMUAccelerationG;
	}

	public static boolean isTilted(Rotation2d imuRoll, Rotation2d imuPitch, Rotation2d minimumTiltIMURoll, Rotation2d minimumTiltIMUPitch) {
		return Math.abs(imuRoll.getRadians()) >= minimumTiltIMURoll.getRadians()
			|| Math.abs(imuPitch.getRadians()) >= minimumTiltIMUPitch.getRadians();
	}

	public static boolean areModulesSkidding(
		SwerveDriveKinematics kinematics,
		SwerveModuleVelocity[] moduleVelocities,
		double minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond,
		double maximumNegligibleVectorNorm
	) {
		ChassisVelocities swerveVelocity = kinematics.toChassisVelocities(moduleVelocities);
		Translation2d swerveTranslationalVelocityMetersPerSecond = new Translation2d(swerveVelocity.vx, swerveVelocity.vy);

		SwerveModuleVelocity[] moduleRotationalVelocities = kinematics
			.toSwerveModuleVelocities(new ChassisVelocities(0, 0, swerveVelocity.omega));
		SwerveModuleVelocity[] moduleTranslationalVelocities = getModuleTranslationalVelocities(
			moduleVelocities,
			moduleRotationalVelocities,
			maximumNegligibleVectorNorm
		);

		for (SwerveModuleVelocity moduleTranslationalVelocity : moduleTranslationalVelocities) {
			if (
				!ToleranceMath.isNear(
					swerveTranslationalVelocityMetersPerSecond,
					new Translation2d(moduleTranslationalVelocity.velocity, moduleTranslationalVelocity.angle),
					minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond
				)
			) {
				return true;
			}
		}
		return false;
	}

	private static SwerveModuleVelocity[] getModuleTranslationalVelocities(
		SwerveModuleVelocity[] moduleVelocities,
		SwerveModuleVelocity[] moduleRotationalVelocities,
		double maximumNegligibleVectorNorm
	) {
		SwerveModuleVelocity[] moduleTranslationalVelocities = new SwerveModuleVelocity[Math
			.min(moduleVelocities.length, moduleRotationalVelocities.length)];
		for (int i = 0; i < moduleTranslationalVelocities.length; i++) {
			moduleTranslationalVelocities[i] = getModuleTranslationalVelocity(
				moduleVelocities[i],
				moduleRotationalVelocities[i],
				maximumNegligibleVectorNorm
			);
		}
		return moduleTranslationalVelocities;
	}

	private static SwerveModuleVelocity getModuleTranslationalVelocity(
		SwerveModuleVelocity moduleVelocity,
		SwerveModuleVelocity moduleRotationalVelocity,
		double maximumNegligibleVectorNorm
	) {
		Translation2d velocityDifference = new Translation2d(moduleVelocity.velocity, moduleVelocity.angle)
			.minus(new Translation2d(moduleRotationalVelocity.velocity, moduleRotationalVelocity.angle));
		return velocityDifference.getNorm() > maximumNegligibleVectorNorm
			? new SwerveModuleVelocity(velocityDifference.getNorm(), velocityDifference.getAngle())
			: new SwerveModuleVelocity();
	}

}
