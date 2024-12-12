package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Field;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.module.ModuleUtils;

import java.util.Optional;
import java.util.function.Supplier;


public class SwerveStateHelper {

	private final Swerve swerve;
	private final SwerveConstants swerveConstants;
	private final Supplier<Optional<Pose2d>> robotPoseSupplier;
	private final Supplier<Optional<Translation2d>> noteTranslationSupplier;

	public SwerveStateHelper(
		Supplier<Optional<Pose2d>> robotPoseSupplier,
		Supplier<Optional<Translation2d>> noteTranslationSupplier,
		Swerve swerve
	) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = robotPoseSupplier;
		this.noteTranslationSupplier = noteTranslationSupplier;
	}

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		return switch (swerveState.getAimAssist()) {
			case NONE -> speeds;
			case SPEAKER -> handleSpeakerAssist(speeds, robotPoseSupplier.get());
			case NOTE -> handleNoteAimAssist(speeds, robotPoseSupplier.get(), noteTranslationSupplier.get(), swerveState);
			case AMP -> handleAmpAssist(speeds, robotPoseSupplier.get());
		};
	}

	private ChassisSpeeds handleNoteAimAssist(
		ChassisSpeeds speeds,
		Optional<Pose2d> optionalRobotPose,
		Optional<Translation2d> optionalNoteTranslation,
		SwerveState swerveState
	) {
		if (optionalRobotPose.isEmpty() || optionalNoteTranslation.isEmpty()) {
			return speeds;
		}
		return AimAssistMath
			.getObjectAssistedSpeeds(speeds, optionalRobotPose.get(), optionalNoteTranslation.get(), swerveConstants, swerveState);
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
			SwerveMath.getRelativeTranslation(robotPose.getTranslation(), Field.getSpeaker().toTranslation2d()).getAngle(),
			swerveConstants
		);
	}


	public Translation2d getRotationAxis(RotateAxis rotationAxisState) {
		return switch (rotationAxisState) {
			case MIDDLE_OF_ROBOT -> new Translation2d();
			case FRONT_LEFT_MODULE -> swerveConstants.modulesLocations()[ModuleUtils.ModulePosition.FRONT_LEFT.getIndex()];
			case FRONT_RIGHT_MODULE -> swerveConstants.modulesLocations()[ModuleUtils.ModulePosition.FRONT_RIGHT.getIndex()];
			case BACK_LEFT_MODULE -> swerveConstants.modulesLocations()[ModuleUtils.ModulePosition.BACK_LEFT.getIndex()];
			case BACK_RIGHT_MODULE -> swerveConstants.modulesLocations()[ModuleUtils.ModulePosition.BACK_RIGHT.getIndex()];
		};
	}

	public RotateAxis getFarRotateAxis(boolean isLeft) {
		Rotation2d currentAllianceHeading = swerve.getAllianceRelativeHeading();
		if (Math.abs(currentAllianceHeading.getDegrees()) <= MathConstants.EIGHTH_CIRCLE.getDegrees()) { // -45 <= x <= 45
			return isLeft ? RotateAxis.FRONT_LEFT_MODULE : RotateAxis.FRONT_RIGHT_MODULE;
		}
		if (Math.abs(currentAllianceHeading.getDegrees()) >= MathConstants.EIGHTH_CIRCLE.getDegrees() * 3) { // -135 - x - 135
			return isLeft ? RotateAxis.BACK_RIGHT_MODULE : RotateAxis.BACK_LEFT_MODULE;
		}
		if (currentAllianceHeading.getDegrees() > 0) { // 45 <= x <= 135
			return isLeft ? RotateAxis.FRONT_RIGHT_MODULE : RotateAxis.BACK_RIGHT_MODULE;
		}
		return isLeft ? RotateAxis.BACK_LEFT_MODULE : RotateAxis.FRONT_LEFT_MODULE; // -45 >= x >= -135
	}

	public RotateAxis getFarRightRotateAxis() {
		return getFarRotateAxis(false);
	}

	public RotateAxis getFarLeftRotateAxis() {
		return getFarRotateAxis(true);
	}

}
