package frc.utils.pose;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.utils.math.ToleranceMath;

public class PoseUtil {

	public static boolean getIsAccelerationHigh(Translation2d imuAccelerationG, double minimumIMUAccelerationG) {
		return imuAccelerationG.getNorm() >= minimumIMUAccelerationG;
	}

	public static boolean getIsTilted(Rotation2d imuRoll, Rotation2d imuPitch, Rotation2d minimumTiltIMURoll, Rotation2d minimumTiltIMUPitch) {
		return Math.abs(imuRoll.getRadians()) >= minimumTiltIMURoll.getRadians()
			|| Math.abs(imuPitch.getRadians()) >= minimumTiltIMUPitch.getRadians();
	}

	public static boolean getAreModulesSkidding(
		SwerveDriveKinematics kinematics,
		SwerveModuleState[] moduleStates,
		double minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond,
		double maximumNegligibleVectorNorm
	) {
		ChassisSpeeds swerveVelocity = kinematics.toChassisSpeeds(moduleStates);
		Translation2d swerveTranslationalVelocityMetersPerSecond = new Translation2d(
			swerveVelocity.vxMetersPerSecond,
			swerveVelocity.vyMetersPerSecond
		);

		SwerveModuleState[] moduleRotationalStates = kinematics
			.toSwerveModuleStates(new ChassisSpeeds(0, 0, swerveVelocity.omegaRadiansPerSecond));
		SwerveModuleState[] moduleTranslationalStates = getModuleTranslationalStates(
			moduleStates,
			moduleRotationalStates,
			maximumNegligibleVectorNorm
		);

		for (SwerveModuleState moduleTranslationalState : moduleTranslationalStates) {
			if (
				!ToleranceMath.isNear(
					swerveTranslationalVelocityMetersPerSecond,
					new Translation2d(moduleTranslationalState.speedMetersPerSecond, moduleTranslationalState.angle),
					minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond
				)
			) {
				return true;
			}
		}
		return false;
	}

	private static SwerveModuleState[] getModuleTranslationalStates(
		SwerveModuleState[] moduleStates,
		SwerveModuleState[] moduleRotationalStates,
		double maximumNegligibleVectorNorm
	) {
		SwerveModuleState[] moduleTranslationalStates = new SwerveModuleState[Math.min(moduleStates.length, moduleRotationalStates.length)];
		for (int i = 0; i < moduleTranslationalStates.length; i++) {
			moduleTranslationalStates[i] = getModuleTranslationalState(moduleStates[i], moduleRotationalStates[i], maximumNegligibleVectorNorm);
		}
		return moduleTranslationalStates;
	}

	private static SwerveModuleState getModuleTranslationalState(
		SwerveModuleState moduleState,
		SwerveModuleState moduleRotationalState,
		double maximumNegligibleVectorNorm
	) {
		Translation2d velocityDifference = new Translation2d(moduleState.speedMetersPerSecond, moduleState.angle)
			.minus(new Translation2d(moduleRotationalState.speedMetersPerSecond, moduleRotationalState.angle));
		return velocityDifference.getNorm() > maximumNegligibleVectorNorm
			? new SwerveModuleState(velocityDifference.getNorm(), velocityDifference.getAngle())
			: new SwerveModuleState();
	}

}
