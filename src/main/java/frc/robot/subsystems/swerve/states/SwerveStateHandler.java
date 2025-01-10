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
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.subsystems.swerve.states.aimassist.AimAssistMath;
import frc.utils.math.PoseMath;

import java.util.Optional;
import java.util.function.Supplier;

public class SwerveStateHandler {

	private final Swerve swerve;
	private final SwerveConstants swerveConstants;
	private Optional<Supplier<Pose2d>> robotPoseSupplier;
	private Supplier<Optional<Translation2d>> objectTranslationSupplier;

	public SwerveStateHandler(Swerve swerve) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = Optional.empty();
		this.objectTranslationSupplier = Optional::empty;
	}

	public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
		this.robotPoseSupplier = Optional.of(robotPoseSupplier);
	}

	public void setObjectTranslationSupplier(Supplier<Optional<Translation2d>> objectTranslationSupplier) {
		this.objectTranslationSupplier = objectTranslationSupplier;
	}

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getAimAssist() == AimAssist.NONE) {
			return speeds;
		}
		if (swerveState.getAimAssist() == AimAssist.AMP) {
			Rotation2d robotHeading = robotPoseSupplier.isPresent() ? robotPoseSupplier.get().get().getRotation() : swerve.getAbsoluteHeading();
			return handleAmpAssist(speeds, robotHeading);
		}
		if (swerveState.getAimAssist() == AimAssist.SPEAKER && robotPoseSupplier.isPresent()) {
			return handleSpeakerAssist(speeds, robotPoseSupplier.get().get());
		}
		if (swerveState.getAimAssist() == AimAssist.NONE && robotPoseSupplier.isPresent() && objectTranslationSupplier.get().isPresent()) {
			return handleNoteAimAssist(speeds, robotPoseSupplier.get().get(), objectTranslationSupplier.get().get(), swerveState);
		}

		return speeds;
	}

	private ChassisSpeeds handleNoteAimAssist(ChassisSpeeds speeds, Pose2d robotPose, Translation2d objectTranslation, SwerveState swerveState) {
		return AimAssistMath.getObjectAssistedSpeeds(speeds, robotPose, objectTranslation, swerveConstants, swerveState);
	}

	private ChassisSpeeds handleAmpAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading) {
		return AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotHeading, Field.getAngleToAmp(), swerveConstants);
	}

	private ChassisSpeeds handleSpeakerAssist(ChassisSpeeds speeds, Pose2d robotPose) {
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
