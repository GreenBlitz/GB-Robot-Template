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
import frc.robot.subsystems.swerve.modules.ModuleUtils;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.swervestatehelpers.AimAssistMath.getObjectAssistedSpeeds;
import static frc.robot.subsystems.swerve.swervestatehelpers.AimAssistMath.getRotationAssistedChassisSpeeds;

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

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(AimAssist aimAssist, ChassisSpeeds chassisSpeeds, SwerveState state) {
		if (robotPoseSupplier.get().isEmpty()) {
			return switch (aimAssist) {
				case AMP -> handleAmpAssist(chassisSpeeds, swerve.getAbsoluteHeading());
				default -> chassisSpeeds;
			};
		}
		Pose2d robotPose = robotPoseSupplier.get().get();
		return switch (aimAssist) {
			case SPEAKER -> handleSpeakerAssist(chassisSpeeds, robotPose);
			case AMP -> handleAmpAssist(chassisSpeeds, robotPose.getRotation());
			case NOTE -> handleNoteAimAssist(chassisSpeeds, robotPose, noteTranslationSupplier.get().get(), state);
			case NONE -> chassisSpeeds;
		};
	}

	private ChassisSpeeds handleNoteAimAssist(ChassisSpeeds chassisSpeeds, Pose2d robotPose, Translation2d noteTranslation, SwerveState state) {
		return getObjectAssistedSpeeds(chassisSpeeds, robotPose, noteTranslation, swerveConstants, state);
	}

	private ChassisSpeeds handleAmpAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotAngle) {
		return getRotationAssistedChassisSpeeds(chassisSpeeds, robotAngle, Field.getAngleToAmp(), swerveConstants);
	}

	private ChassisSpeeds handleSpeakerAssist(ChassisSpeeds chassisSpeeds, Pose2d robotPose) {
		return getRotationAssistedChassisSpeeds(
			chassisSpeeds,
			robotPose.getRotation(),
			SwerveMath.getRelativeTranslation(robotPose, Field.getSpeaker().toTranslation2d()).getAngle(),
			swerveConstants
		);
	}

	public Translation2d getRotationAxis(RotateAxis rotationAxisState) {
		return switch (rotationAxisState) {
			case MIDDLE_OF_ROBOT -> new Translation2d();
			case FRONT_LEFT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModulePosition.FRONT_LEFT.getIndex()];
			case FRONT_RIGHT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModulePosition.FRONT_RIGHT.getIndex()];
			case BACK_LEFT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModulePosition.BACK_LEFT.getIndex()];
			case BACK_RIGHT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModulePosition.BACK_RIGHT.getIndex()];
		};
	}

	public RotateAxis getFarRotateAxis(boolean isLeft) {
		Rotation2d currentAllianceAngle = swerve.getAllianceRelativeHeading();
		if (Math.abs(currentAllianceAngle.getDegrees()) <= MathConstants.EIGHTH_CIRCLE.getDegrees()) { // -45 <= x <= 45
			return isLeft ? RotateAxis.FRONT_LEFT_MODULE : RotateAxis.FRONT_RIGHT_MODULE;
		}
		if (Math.abs(currentAllianceAngle.getDegrees()) >= MathConstants.EIGHTH_CIRCLE.getDegrees() * 3) { // -135 - x - 135
			return isLeft ? RotateAxis.BACK_RIGHT_MODULE : RotateAxis.BACK_LEFT_MODULE;
		}
		if (currentAllianceAngle.getDegrees() > 0) { // 45 <= x <= 135
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
