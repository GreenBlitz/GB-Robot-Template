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
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.swervestatehelpers.AimAssistUtils.getRotationAssistedSpeeds;

public class SwerveStateHelper {

	private final Swerve swerve;
	private final SwerveConstants swerveConstants;
	private final Supplier<Pose2d> robotPoseSupplier;
	private final Supplier<Optional<Translation2d>> noteTranslationSupplier;

	public SwerveStateHelper(
		Supplier<Pose2d> robotPoseSupplier,
		Supplier<Optional<Translation2d>> noteTranslationSupplier,
		Swerve swerve
	) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = robotPoseSupplier;
		this.noteTranslationSupplier = noteTranslationSupplier;
	}

	public ChassisSpeeds applyAimAssistOnSpeeds(AimAssist aimAssistState, ChassisSpeeds chassisSpeeds) {
		return switch (aimAssistState) {
			case SPEAKER ->
				getRotationAssistedSpeeds(
					chassisSpeeds,
					robotPoseSupplier.get().getRotation(),
					SwerveMath.getRelativeTranslation(robotPoseSupplier.get().getTranslation(), new Translation2d(0, 0))
						.getAngle(),
					swerveConstants
				);
			case AMP ->
				getRotationAssistedSpeeds(
					chassisSpeeds,
					robotPoseSupplier.get().getRotation(),
					Field.getAngleToAmp(),
					swerveConstants
				);
			case NOTE -> handleNoteAssistState(chassisSpeeds, robotPoseSupplier, noteTranslationSupplier);
			case NONE -> chassisSpeeds;
		};
	}


	private ChassisSpeeds handleNoteAssistState(
		ChassisSpeeds speeds,
		Supplier<Pose2d> robotPoseSupplier,
		Supplier<Optional<Translation2d>> noteTranslationSupplier
	) {
		if (noteTranslationSupplier.get().isEmpty()) {
			return speeds;
		}

		Translation2d noteRelativeToRobot = SwerveMath
			.getRelativeTranslation(robotPoseSupplier.get(), noteTranslationSupplier.get().get());


		double pidGainHorizontalVelocity = swerveConstants.yMetersPIDController().calculate(0, noteRelativeToRobot.getY());

		Logger.recordOutput("current angle", robotPoseSupplier.get().getRotation().getDegrees());
		Logger.recordOutput("target note", noteTranslationSupplier.get().get());
		return new ChassisSpeeds(
			speeds.vxMetersPerSecond
				+ pidGainHorizontalVelocity * Math.sin(robotPoseSupplier.get().getRotation().unaryMinus().getRadians()),
			speeds.vyMetersPerSecond
				+ pidGainHorizontalVelocity * Math.cos(robotPoseSupplier.get().getRotation().unaryMinus().getRadians()),
			speeds.omegaRadiansPerSecond
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
