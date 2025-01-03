package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.MathConstants;
import frc.constants.field.Field;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.states.aimassist.AimAssistMath;
import frc.utils.math.PoseMath;

import java.util.Optional;
import java.util.function.Supplier;

public class SwerveStateHelper {

	private final Swerve swerve;
	private final SwerveConstants swerveConstants;
	private final Supplier<Optional<Pose2d>> robotPoseSupplier;
	private final Supplier<Optional<Translation2d>> objectTranslationSupplier;

	public SwerveStateHelper(
		Supplier<Optional<Pose2d>> robotPoseSupplier,
		Supplier<Optional<Translation2d>> objectTranslationSupplier,
		Swerve swerve
	) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = robotPoseSupplier;
		this.objectTranslationSupplier = objectTranslationSupplier;
	}

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		return switch (swerveState.getAimAssist()) {
			case NONE -> speeds;
			case SPEAKER -> handleSpeakerAssist(speeds, robotPoseSupplier.get());
			case NOTE -> handleNoteAimAssist(speeds, robotPoseSupplier.get(), objectTranslationSupplier.get(), swerveState);
			case AMP -> handleAmpAssist(speeds, robotPoseSupplier.get());
		};
	}

	private ChassisSpeeds handleNoteAimAssist(
		ChassisSpeeds speeds,
		Optional<Pose2d> optionalRobotPose,
		Optional<Translation2d> optionalObjectTranslation,
		SwerveState swerveState
	) {
		if (optionalRobotPose.isEmpty() || optionalObjectTranslation.isEmpty()) {
			return speeds;
		}
		return AimAssistMath
			.getObjectAssistedSpeeds(speeds, optionalRobotPose.get(), optionalObjectTranslation.get(), swerveConstants, swerveState);
	}

	private ChassisSpeeds handleAmpAssist(ChassisSpeeds chassisSpeeds, Optional<Pose2d> optionalRobotPose) {
		Rotation2d robotHeading = optionalRobotPose.isPresent() ? optionalRobotPose.get().getRotation() : swerve.getAbsoluteHeading();
		return AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotHeading, Field.getAngleToAmp(), swerveConstants);
	}

	private ChassisSpeeds handleSpeakerAssist(ChassisSpeeds speeds, Optional<Pose2d> optionalRobotPose) {
		if (optionalRobotPose.isEmpty()) {
			return speeds;
		}
		Pose2d robotPose = optionalRobotPose.get();
		return AimAssistMath.getRotationAssistedChassisSpeeds(
			speeds,
			robotPose.getRotation(),
			PoseMath.getRelativeTranslation(robotPose.getTranslation(), Field.getSpeaker().toTranslation2d()).getAngle(),
			swerveConstants
		);
	}


	public Translation2d getRotationAxis(RotateAxis rotationAxisState) {
		return switch (rotationAxisState) {
			case MIDDLE_OF_CHASSIS -> new Translation2d();
			case FRONT_LEFT_MODULE -> swerve.getModules().getModule(ModuleUtils.ModulePosition.FRONT_LEFT).getPositionFromCenterMeters();
			case FRONT_RIGHT_MODULE -> swerve.getModules().getModule(ModuleUtils.ModulePosition.FRONT_RIGHT).getPositionFromCenterMeters();
			case BACK_LEFT_MODULE -> swerve.getModules().getModule(ModuleUtils.ModulePosition.BACK_LEFT).getPositionFromCenterMeters();
			case BACK_RIGHT_MODULE -> swerve.getModules().getModule(ModuleUtils.ModulePosition.BACK_RIGHT).getPositionFromCenterMeters();
		};
	}

	public RotateAxis getFarRotateAxis(boolean isLeft) {
		Rotation2d currentAllianceRelativeHeading = swerve.getAllianceRelativeHeading();
		// -45 <= x <= 45
		if (Math.abs(currentAllianceRelativeHeading.getDegrees()) <= MathConstants.EIGHTH_CIRCLE.getDegrees()) {
			return isLeft ? RotateAxis.FRONT_LEFT_MODULE : RotateAxis.FRONT_RIGHT_MODULE;
		}
		// -180 <= x <= -135 || 135 <= x <= 180
		if (Math.abs(currentAllianceRelativeHeading.getDegrees()) >= MathConstants.EIGHTH_CIRCLE.getDegrees() * 3) {
			return isLeft ? RotateAxis.BACK_RIGHT_MODULE : RotateAxis.BACK_LEFT_MODULE;
		}
		// 45 <= x <= 135
		if (currentAllianceRelativeHeading.getDegrees() > 0) {
			return isLeft ? RotateAxis.FRONT_RIGHT_MODULE : RotateAxis.BACK_RIGHT_MODULE;
		}
		// -45 >= x >= -135
		return isLeft ? RotateAxis.BACK_LEFT_MODULE : RotateAxis.FRONT_LEFT_MODULE;
	}

	public RotateAxis getFarRightRotateAxis() {
		return getFarRotateAxis(false);
	}

	public RotateAxis getFarLeftRotateAxis() {
		return getFarRotateAxis(true);
	}

}
