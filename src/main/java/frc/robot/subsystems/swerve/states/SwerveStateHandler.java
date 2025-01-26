package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.MathConstants;
import frc.constants.field.Field;
import frc.constants.field.enums.CoralStationPosition;
import frc.constants.field.enums.ReefBranch;
import frc.constants.field.enums.ReefSide;
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

	private Supplier<Optional<ReefSide>> reefSupplier;
	private Supplier<Optional<CoralStationPosition>> feederSupplier;
	private Supplier<Optional<ReefBranch>> branchSupplier;

	public SwerveStateHandler(Swerve swerve) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = Optional.empty();

		this.reefSupplier = Optional::empty;
		this.feederSupplier = Optional::empty;
		this.branchSupplier = Optional::empty;
	}

	public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
		this.robotPoseSupplier = Optional.of(robotPoseSupplier);
	}

	public void setReefSupplier(Supplier<Optional<ReefSide>> reefTranslationSupplier) {
		this.reefSupplier = reefTranslationSupplier;
	}

	public void setFeederSupplier(Supplier<Optional<CoralStationPosition>> feederTranslationSupplier) {
		this.feederSupplier = feederTranslationSupplier;
	}

	public void setBranchSupplier(Supplier<Optional<ReefBranch>> branchTranslationSupplier) {
		this.branchSupplier = branchTranslationSupplier;
	}

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getAimAssist() == AimAssist.NONE) {
			return speeds;
		}
		if (swerveState.getAimAssist() == AimAssist.REEF && robotPoseSupplier.isPresent() && reefSupplier.get().isPresent()) {
			return handleReefAimAssist(speeds, robotPoseSupplier.get().get().getRotation(), reefSupplier.get().get());
		}
		if (swerveState.getAimAssist() == AimAssist.FEEDER && robotPoseSupplier.isPresent() && feederSupplier.get().isPresent()) {
			return handleFeederAimAssist(speeds, robotPoseSupplier.get().get().getRotation(), feederSupplier.get().get());
		}
		if (
			swerveState.getAimAssist() == AimAssist.BRANCH
				&& robotPoseSupplier.isPresent()
				&& branchSupplier.get().isPresent()
				&& reefSupplier.get().isPresent()
		) {
			return handleBranchAimAssist(
				speeds,
				robotPoseSupplier.get().get(),
				branchSupplier.get().get(),
				reefSupplier.get().get(),
				swerveState
			);
		}
		if (swerveState.getAimAssist() == AimAssist.ALGI_REMOVE && robotPoseSupplier.isPresent() && reefSupplier.get().isPresent()) {
			return handleAlgiAimAssist(speeds, robotPoseSupplier.get().get(), reefSupplier.get().get(), swerveState);
		}

		return speeds;
	}

	private ChassisSpeeds handleReefAimAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, ReefSide reefSide) {
		Rotation2d reefSideAngle = Field.getMiddleOfReefSide(reefSide).getRotation();
		return AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotHeading, reefSideAngle, swerveConstants);
	}

	private ChassisSpeeds handleFeederAimAssist(
		ChassisSpeeds chassisSpeeds,
		Rotation2d robotHeading,
		CoralStationPosition coralStationPosition
	) {
		Rotation2d feederAngle = Field.getMiddleOfCoralStation(coralStationPosition).getRotation();
		return AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotHeading, feederAngle, swerveConstants);
	}

	private ChassisSpeeds handleBranchAimAssist(
		ChassisSpeeds chassisSpeeds,
		Pose2d robotPose,
		ReefBranch reefBranch,
		ReefSide reefSide,
		SwerveState swerveState
	) {
		Translation2d branch = Field.getCoralPlacement(reefBranch);
		Rotation2d angleToReefSide = Field.getMiddleOfReefSide(reefSide).getRotation();
		chassisSpeeds = AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotPose.getRotation(), angleToReefSide, swerveConstants);
		robotPose = new Pose2d(robotPose.getX(), robotPose.getY(), angleToReefSide);
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, branch, swerveConstants, swerveState);
	}

	private ChassisSpeeds handleAlgiAimAssist(ChassisSpeeds chassisSpeeds, Pose2d robotPose, ReefSide reefSide, SwerveState swerveState) {
		Pose2d middleOfReefSide = Field.getMiddleOfReefSide(reefSide);
		Rotation2d angleToReefSide = Field.getMiddleOfReefSide(reefSide).getRotation();
		chassisSpeeds = AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotPose.getRotation(), angleToReefSide, swerveConstants);
		robotPose = new Pose2d(robotPose.getX(), robotPose.getY(), angleToReefSide);
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, middleOfReefSide.getTranslation(), swerveConstants, swerveState);
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
