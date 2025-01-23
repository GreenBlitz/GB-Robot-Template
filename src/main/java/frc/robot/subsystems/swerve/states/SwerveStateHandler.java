package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.MathConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.subsystems.swerve.states.aimassist.AimAssistMath;

import java.util.Optional;
import java.util.function.Supplier;

public class SwerveStateHandler {

	private final Swerve swerve;
	private final SwerveConstants swerveConstants;
	private Optional<Supplier<Pose2d>> robotPoseSupplier;

	private Supplier<Optional<Translation2d>> reefTranslationSupplier;
	private Supplier<Optional<Translation2d>> feederTranslationSupplier;
	private Supplier<Optional<Translation2d>> branchTranslationSupplier;
	private Supplier<Optional<Translation2d>> algiTranslationSupplier;
	private Supplier<Optional<Translation2d>> slotTranslationSupplier;

	public SwerveStateHandler(Swerve swerve) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = Optional.empty();

		this.reefTranslationSupplier = Optional::empty;
		this.feederTranslationSupplier = Optional::empty;
		this.branchTranslationSupplier = Optional::empty;
		this.algiTranslationSupplier = Optional::empty;
		this.slotTranslationSupplier = Optional::empty;
	}

	public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
		this.robotPoseSupplier = Optional.of(robotPoseSupplier);
	}

	public void setReedTranslationSupplier(Supplier<Optional<Translation2d>> reefTranslationSupplier) {
		this.reefTranslationSupplier = reefTranslationSupplier;
	}

	public void setFeederTranslationSupplier(Supplier<Optional<Translation2d>> feederTranslationSupplier) {
		this.feederTranslationSupplier = feederTranslationSupplier;
	}

	public void setBranchTranslationSupplier(Supplier<Optional<Translation2d>> branchTranslationSupplier) {
		this.branchTranslationSupplier = branchTranslationSupplier;
	}

	public void setAlgiTranslationSupplier(Supplier<Optional<Translation2d>> algiTranslationSupplier) {
		this.algiTranslationSupplier = algiTranslationSupplier;
	}

	public void setSlotTranslationSupplier(Supplier<Optional<Translation2d>> slotTranslationSupplier) {
		this.slotTranslationSupplier = slotTranslationSupplier;
	}

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getAimAssist() == AimAssist.NONE) {
			return speeds;
		}
		if (swerveState.getAimAssist() == AimAssist.REEF && robotPoseSupplier.isPresent()) {
			return handleReefAimAssist(speeds, robotPoseSupplier.get().get().getRotation());
		}
		if (swerveState.getAimAssist() == AimAssist.FEEDER && robotPoseSupplier.isPresent() && feederTranslationSupplier.get().isPresent()) {
			return handleFeederAimAssist(speeds, robotPoseSupplier.get().get().getRotation());
		}
		if (swerveState.getAimAssist() == AimAssist.BRANCH && robotPoseSupplier.isPresent() && branchTranslationSupplier.get().isPresent()) {
			return handleBranchAimAssist(speeds, robotPoseSupplier.get().get(), branchTranslationSupplier.get().get(), swerveState);
		}
		if (swerveState.getAimAssist() == AimAssist.ALGI_REMOVE && robotPoseSupplier.isPresent() && algiTranslationSupplier.get().isPresent()) {
			return handleAlgiAimAssist(speeds, robotPoseSupplier.get().get(), algiTranslationSupplier.get().get(), swerveState);
		}
		if (swerveState.getAimAssist() == AimAssist.SLOT && robotPoseSupplier.isPresent() && slotTranslationSupplier.get().isPresent()) {
			return handleSlotAimAssist(speeds, robotPoseSupplier.get().get(), slotTranslationSupplier.get().get(), swerveState);
		}

		return speeds;
	}

	private ChassisSpeeds handleReefAimAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading) {
		return AimAssistMath
			.getRotationAssistedChassisSpeeds(chassisSpeeds, robotHeading, reefTranslationSupplier.get().get().getAngle(), swerveConstants);
	}

	private ChassisSpeeds handleFeederAimAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading) {
		return AimAssistMath
			.getRotationAssistedChassisSpeeds(chassisSpeeds, robotHeading, feederTranslationSupplier.get().get().getAngle(), swerveConstants);
	}

	private ChassisSpeeds handleBranchAimAssist(
		ChassisSpeeds chassisSpeeds,
		Pose2d robotPose,
		Translation2d branchTranslation,
		SwerveState swerveState
	) {
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, branchTranslation, swerveConstants, swerveState);
	}

	private ChassisSpeeds handleAlgiAimAssist(
		ChassisSpeeds chassisSpeeds,
		Pose2d robotPose,
		Translation2d algiTranslation,
		SwerveState swerveState
	) {
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, algiTranslation, swerveConstants, swerveState);
	}

	private ChassisSpeeds handleSlotAimAssist(
		ChassisSpeeds chassisSpeeds,
		Pose2d robotPose,
		Translation2d slotTranslation,
		SwerveState swerveState
	) {
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, slotTranslation, swerveConstants, swerveState);
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
