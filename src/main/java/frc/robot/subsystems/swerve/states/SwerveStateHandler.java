package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.MathConstants;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.ReefSide;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.subsystems.swerve.states.aimassist.AimAssistMath;

import java.util.Optional;
import java.util.function.Supplier;

public class SwerveStateHandler {

	private final Swerve swerve;
	private final SwerveConstants swerveConstants;
	private Optional<Supplier<Pose2d>> robotPoseSupplier;

	private Supplier<Optional<ReefSide>> reefSideSupplier;
	private Supplier<Optional<CoralStation>> feederSupplier;
	private Supplier<Optional<Branch>> branchSupplier;

	public SwerveStateHandler(Swerve swerve) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = Optional.empty();

		this.reefSideSupplier = Optional::empty;
		this.feederSupplier = Optional::empty;
		this.branchSupplier = Optional::empty;
	}

	public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
		this.robotPoseSupplier = Optional.of(robotPoseSupplier);
	}

	public void setReefSideSupplier(Supplier<Optional<ReefSide>> reefSideSupplier) {
		this.reefSideSupplier = reefSideSupplier;
	}

	public void setFeederSupplier(Supplier<Optional<CoralStation>> feederSupplier) {
		this.feederSupplier = feederSupplier;
	}

	public void setBranchSupplier(Supplier<Optional<Branch>> branchSupplier) {
		this.branchSupplier = branchSupplier;
	}

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getAimAssist() == AimAssist.NONE) {
			return speeds;
		}
		if (swerveState.getAimAssist() == AimAssist.REEF && robotPoseSupplier.isPresent() && reefSideSupplier.get().isPresent()) {
			return handleReefAimAssist(speeds, robotPoseSupplier.get().get().getRotation(), reefSideSupplier.get().get());
		}
		if (swerveState.getAimAssist() == AimAssist.FEEDER && robotPoseSupplier.isPresent() && feederSupplier.get().isPresent()) {
			return handleFeederAimAssist(speeds, robotPoseSupplier.get().get().getRotation(), feederSupplier.get().get());
		}
		if (swerveState.getAimAssist() == AimAssist.BRANCH && robotPoseSupplier.isPresent() && branchSupplier.get().isPresent()) {
			return handleBranchAimAssist(speeds, robotPoseSupplier.get().get(), branchSupplier.get().get(), swerveState);
		}
		if (swerveState.getAimAssist() == AimAssist.ALGI_REMOVE && robotPoseSupplier.isPresent() && reefSideSupplier.get().isPresent()) {
			return handleAlgiAimAssist(speeds, robotPoseSupplier.get().get(), reefSideSupplier.get().get(), swerveState);
		}

		return speeds;
	}

	private ChassisSpeeds handleReefAimAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, ReefSide reefSide) {
		Rotation2d reefSideAngle = Field.getMiddleOfReefSide(reefSide).getRotation();
		return AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotHeading, reefSideAngle, swerveConstants);
	}

	private ChassisSpeeds handleFeederAimAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, CoralStation coralStation) {
		Rotation2d feederAngle = Field.getMiddleOfCoralStation(coralStation).getRotation();
		return AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotHeading, feederAngle, swerveConstants);
	}

	private ChassisSpeeds handleBranchAimAssist(ChassisSpeeds chassisSpeeds, Pose2d robotPose, Branch reefBranch, SwerveState swerveState) {
		Translation2d branch = Field.getCoralPlacement(reefBranch);
		Rotation2d angleToReefSide = Field.getMiddleOfReefSide(reefBranch.getReefSide()).getRotation();
		chassisSpeeds = AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotPose.getRotation(), angleToReefSide, swerveConstants);
		Pose2d poseWithHeading = new Pose2d(robotPose.getX(), robotPose.getY(), angleToReefSide.plus(MathConstants.HALF_CIRCLE));
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, poseWithHeading, branch, swerveConstants, swerveState);
	}

	private ChassisSpeeds handleAlgiAimAssist(ChassisSpeeds chassisSpeeds, Pose2d robotPose, ReefSide reefSide, SwerveState swerveState) {
		Pose2d middleOfReefSide = Field.getMiddleOfReefSide(reefSide);
		Rotation2d angleToReefSide = Field.getMiddleOfReefSide(reefSide).getRotation();
		chassisSpeeds = AimAssistMath.getRotationAssistedChassisSpeeds(chassisSpeeds, robotPose.getRotation(), angleToReefSide, swerveConstants);
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, middleOfReefSide.getTranslation(), swerveConstants, swerveState);
	}

	public Translation2d getRotationAxis(RotateAxis rotationAxisState) {
		return switch (rotationAxisState) {
			case MIDDLE_OF_CHASSIS -> new Translation2d();
			case FRONT_LEFT_MODULE -> swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getPositionFromCenterMeters();
			case FRONT_RIGHT_MODULE -> swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_RIGHT).getPositionFromCenterMeters();
			case BACK_LEFT_MODULE -> swerve.getModules().getModule(ModuleUtil.ModulePosition.BACK_LEFT).getPositionFromCenterMeters();
			case BACK_RIGHT_MODULE -> swerve.getModules().getModule(ModuleUtil.ModulePosition.BACK_RIGHT).getPositionFromCenterMeters();
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
