package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.states.DriveSpeed;
import frc.utils.math.ToleranceMath;
import frc.utils.time.TimeUtil;

public class SwerveMath {

	public static double calculateDriveRadiusMeters(Translation2d[] modulePositionsFromCenterMeters) {
		double sum = 0;
		for (Translation2d modulePositionFromCenterMeters : modulePositionsFromCenterMeters) {
			sum += modulePositionFromCenterMeters.getDistance(new Translation2d());
		}
		return sum / modulePositionsFromCenterMeters.length;
	}

	public static ChassisSpeeds allianceToRobotRelativeSpeeds(ChassisSpeeds allianceRelativeSpeeds, Rotation2d allianceRelativeHeading) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(allianceRelativeSpeeds, allianceRelativeHeading);
	}

	public static ChassisSpeeds robotToAllianceRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds, Rotation2d allianceRelativeHeading) {
		return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, allianceRelativeHeading);
	}

	public static ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
		return ChassisSpeeds.discretize(chassisSpeeds, TimeUtil.getLatestCycleTimeSeconds());
	}

	public static ChassisSpeeds powersToSpeeds(ChassisPowers powers, SwerveConstants constants) {
		return new ChassisSpeeds(
			powers.xPower * constants.velocityAt12VoltsMetersPerSecond(),
			powers.yPower * constants.velocityAt12VoltsMetersPerSecond(),
			powers.rotationalPower * constants.maxRotationalVelocityPerSecond().getRadians()
		);
	}

	public static ChassisSpeeds factorSpeeds(ChassisSpeeds speeds, DriveSpeed driveSpeed) {
		return new ChassisSpeeds(
			speeds.vxMetersPerSecond * driveSpeed.getTranslationSpeedFactor(),
			speeds.vyMetersPerSecond * driveSpeed.getTranslationSpeedFactor(),
			speeds.omegaRadiansPerSecond * driveSpeed.getRotationSpeedFactor()
		);
	}

	public static ChassisSpeeds applyDeadband(ChassisSpeeds chassisSpeeds, Pose2d deadbands) {
		double xVelocityMetersPerSecond = ToleranceMath.applyDeadband(chassisSpeeds.vxMetersPerSecond, deadbands.getX());
		double yVelocityMetersPerSecond = ToleranceMath.applyDeadband(chassisSpeeds.vyMetersPerSecond, deadbands.getY());
		double rotationalVelocityRadiansPerSecond = ToleranceMath
			.applyDeadband(chassisSpeeds.omegaRadiansPerSecond, deadbands.getRotation().getRadians());

		return new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationalVelocityRadiansPerSecond);
	}

	private static Translation2d getSwerveTranslationalMotionAccountableModuleVelocity(
		int moduleIndex,
		SwerveModuleState[] swerveRotationalAccountableModuleStates,
		SwerveModuleState[] moduleStates
	) {
		SwerveModuleState swerveRotationalAccountableModuleState = swerveRotationalAccountableModuleStates[moduleIndex];
		SwerveModuleState moduleState = moduleStates[moduleIndex];

		return new Translation2d(
			moduleState.speedMetersPerSecond - swerveRotationalAccountableModuleState.speedMetersPerSecond,
			moduleState.angle.minus(swerveRotationalAccountableModuleState.angle)
		);
	}

	public static Translation2d[] getSwerveTranslationalMotionAccountableModuleVelocities(
		int numOfModules,
		SwerveModuleState[] swerveRotationalAccountableModuleStates,
		SwerveModuleState[] moduleStates
	) {
		Translation2d[] swerveTranslationalMotionAccountableModuleVelocities = new Translation2d[numOfModules];
		for (int i = 0; i < ModuleUtil.ModulePosition.values().length; i++) {
			swerveTranslationalMotionAccountableModuleVelocities[i] = getSwerveTranslationalMotionAccountableModuleVelocity(
				i,
				swerveRotationalAccountableModuleStates,
				moduleStates
			);
		}
		return swerveTranslationalMotionAccountableModuleVelocities;
	}

	public static boolean getAreModulesSkidding(
		ChassisSpeeds robotRelativeVelocity,
		SwerveDriveKinematics kinematics,
		SwerveModuleState[] moduleStates,
		double skidVelocityToleranceMetersPerSecond
	) {
		int numberOfModules = kinematics.getModules().length;
		SwerveModuleState[] swerveRotationalAccountableModuleStates = kinematics
			.toSwerveModuleStates(new ChassisSpeeds(0, 0, robotRelativeVelocity.omegaRadiansPerSecond), new Translation2d());
		Translation2d[] swerveTranslationalMotionAccountableModuleVelocities = getSwerveTranslationalMotionAccountableModuleVelocities(
			numberOfModules,
			swerveRotationalAccountableModuleStates,
			moduleStates
		);
		Translation2d robotTranslationalVelocityMetersPerSecond = new Translation2d(
			robotRelativeVelocity.vxMetersPerSecond,
			robotRelativeVelocity.vyMetersPerSecond
		);
		for (Translation2d swerveTranslationalMotionAccountableModuleVelocity : swerveTranslationalMotionAccountableModuleVelocities) {
			if (
				!ToleranceMath.isNear(
					robotTranslationalVelocityMetersPerSecond,
					swerveTranslationalMotionAccountableModuleVelocity,
					skidVelocityToleranceMetersPerSecond
				)
			) {
				return true;
			}
		}
		return false;
	}

	public static boolean isStill(ChassisSpeeds chassisSpeeds, Pose2d deadbands) {
		return Math.abs(chassisSpeeds.vxMetersPerSecond) <= deadbands.getX()
			&& Math.abs(chassisSpeeds.vyMetersPerSecond) <= deadbands.getY()
			&& Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= deadbands.getRotation().getRadians();
	}

	public static double getDriveMagnitude(ChassisSpeeds chassisSpeeds) {
		return Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
	}

}
