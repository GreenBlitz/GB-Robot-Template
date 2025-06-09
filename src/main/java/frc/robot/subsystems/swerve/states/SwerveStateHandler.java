package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.MathConstants;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.Cage;
import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.CoralStationSlot;
import frc.constants.field.enums.ReefSide;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.subsystems.swerve.states.aimassist.AimAssistMath;
import frc.utils.alerts.Alert;
import frc.utils.math.FieldMath;

import java.util.Optional;
import java.util.function.Supplier;

public class SwerveStateHandler {

	private final Swerve swerve;
	private final SwerveConstants swerveConstants;
	private Optional<Supplier<Pose2d>> robotPoseSupplier;

	private Supplier<Optional<ReefSide>> reefSideSupplier;
	private Supplier<Optional<CoralStation>> coralStationSupplier;
	private Supplier<Optional<Branch>> branchSupplier;
	private Supplier<Optional<CoralStationSlot>> coralStationSlotSupplier;
	private Supplier<Optional<Cage>> cageSupplier;
	private Supplier<Optional<Translation2d>> closestAlgaeSupplier;

	public SwerveStateHandler(Swerve swerve) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = Optional.empty();

		this.reefSideSupplier = Optional::empty;
		this.coralStationSupplier = Optional::empty;
		this.branchSupplier = Optional::empty;
		this.coralStationSlotSupplier = Optional::empty;
		this.cageSupplier = Optional::empty;
		this.closestAlgaeSupplier = Optional::empty;
	}

	public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
		this.robotPoseSupplier = Optional.of(robotPoseSupplier);
	}

	public void setReefSideSupplier(Supplier<Optional<ReefSide>> reefSideSupplier) {
		this.reefSideSupplier = reefSideSupplier;
	}

	public void setCoralStationSupplier(Supplier<Optional<CoralStation>> coralStationSupplier) {
		this.coralStationSupplier = coralStationSupplier;
	}

	public void setBranchSupplier(Supplier<Optional<Branch>> branchSupplier) {
		this.branchSupplier = branchSupplier;
	}

	public void setCoralStationSlotSupplier(Supplier<Optional<CoralStationSlot>> coralStationSlotSupplier) {
		this.coralStationSlotSupplier = coralStationSlotSupplier;
	}

	public void setCageSupplier(Supplier<Optional<Cage>> cageSupplier) {
		this.cageSupplier = cageSupplier;
	}

	public void setClosestAlgaeSupplier(Supplier<Optional<Translation2d>> closestAlgaeSupplier) {
		this.closestAlgaeSupplier = closestAlgaeSupplier;
	}

	private void reportMissingSupplier(String supplierName) {
		new Alert(Alert.AlertType.WARNING, swerve.getLogPath() + "/AimAssist/missing " + supplierName + " supplier").report();
	}

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getAimAssist() == AimAssist.NONE) {
			return speeds;
		}

		if (robotPoseSupplier.isEmpty()) {
			reportMissingSupplier("robot pose");
			return speeds;
		}

		if (swerveState.getAimAssist() == AimAssist.REEF) {
			if (reefSideSupplier.get().isPresent()) {
				return handleReefAimAssist(speeds, robotPoseSupplier.get().get().getRotation(), reefSideSupplier.get().get());
			} else {
				reportMissingSupplier("reef side");
				return speeds;
			}
		}

		if (swerveState.getAimAssist() == AimAssist.CORAL_STATION) {
			if (coralStationSupplier.get().isPresent()) {
				return handleCoralStationAimAssist(speeds, robotPoseSupplier.get().get().getRotation(), coralStationSupplier.get().get());
			} else {
				reportMissingSupplier("coral station");
				return speeds;
			}
		}

		if (swerveState.getAimAssist() == AimAssist.BRANCH) {
			if (branchSupplier.get().isPresent()) {
				return handleBranchAimAssist(speeds, robotPoseSupplier.get().get(), branchSupplier.get().get(), swerveState);
			} else {
				reportMissingSupplier("branch");
				return speeds;
			}
		}

		if (swerveState.getAimAssist() == AimAssist.ALGAE_REMOVE) {
			if (reefSideSupplier.get().isPresent()) {
				return handleAlgaeRemoveAimAssist(speeds, robotPoseSupplier.get().get(), reefSideSupplier.get().get(), swerveState);
			} else {
				reportMissingSupplier("reef side");
				return speeds;
			}
		}

		if (swerveState.getAimAssist() == AimAssist.ALGAE_INTAKE) {
			if (closestAlgaeSupplier.get().isPresent()) {
				return handleAlgaeIntakeAimAssist(speeds, robotPoseSupplier.get().get(), closestAlgaeSupplier.get().get(), swerveState);
			} else {
				reportMissingSupplier("detected algae");
				return speeds;
			}
		}

		if (swerveState.getAimAssist() == AimAssist.CAGE_ROTATION) {
			return handleAngleCageAssist(speeds, robotPoseSupplier.get().get().getRotation());
		}

		if (swerveState.getAimAssist() == AimAssist.CORAL_STATION_SLOT) {
			if (coralStationSlotSupplier.get().isPresent()) {
				return handleCoralStationSlotAimAssist(speeds, robotPoseSupplier.get().get(), coralStationSlotSupplier.get().get(), swerveState);
			} else {
				reportMissingSupplier("coral station slots");
				return speeds;
			}
		}

		if (swerveState.getAimAssist() == AimAssist.CAGE) {
			if (cageSupplier.get().isPresent()) {
				return handleCageAimAssist(speeds, robotPoseSupplier.get().get(), cageSupplier.get().get(), swerveState);
			} else {
				reportMissingSupplier("cage");
				return speeds;
			}
		}

		return speeds;
	}

	private ChassisSpeeds handleReefAimAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, ReefSide reefSide) {
		Rotation2d headingToReefSide = Field.getReefSideMiddle(reefSide).getRotation();
		return AimAssistMath.getRotationAssistedSpeeds(chassisSpeeds, robotHeading, headingToReefSide, swerveConstants);
	}

	private ChassisSpeeds handleCoralStationAimAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, CoralStation coralStation) {
		Rotation2d headingToCoralStation = Field.getCoralStationMiddle(coralStation).getRotation();
		return AimAssistMath.getRotationAssistedSpeeds(chassisSpeeds, robotHeading, headingToCoralStation, swerveConstants);
	}

	private ChassisSpeeds handleBranchAimAssist(ChassisSpeeds chassisSpeeds, Pose2d robotPose, Branch reefBranch, SwerveState swerveState) {
		Translation2d branch = ScoringHelpers.getRobotBranchScoringPose(reefBranch, 0).getTranslation();
		Rotation2d headingToReefSide = Field.getReefSideMiddle(reefBranch.getReefSide()).getRotation();

		chassisSpeeds = AimAssistMath.getRotationAssistedSpeeds(chassisSpeeds, robotPose.getRotation(), headingToReefSide, swerveConstants);
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, headingToReefSide, branch, swerveConstants, swerveState);
	}

	private ChassisSpeeds handleCoralStationSlotAimAssist(
		ChassisSpeeds chassisSpeeds,
		Pose2d robotPose,
		CoralStationSlot coralStationSlot,
		SwerveState swerveState
	) {
		Translation2d slot = ScoringHelpers.getIntakePose(coralStationSlot).getTranslation();
		Rotation2d headingToCoralStation = Field.getCoralStationSlot(coralStationSlot).getRotation();

		chassisSpeeds = AimAssistMath.getRotationAssistedSpeeds(chassisSpeeds, robotPose.getRotation(), headingToCoralStation, swerveConstants);
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, headingToCoralStation, slot, swerveConstants, swerveState);
	}

	private ChassisSpeeds handleAlgaeRemoveAimAssist(ChassisSpeeds chassisSpeeds, Pose2d robotPose, ReefSide reefSide, SwerveState swerveState) {
		Translation2d algaeRemovePose = ScoringHelpers.getAlgaeRemovePose(false).getTranslation();
		Rotation2d headingToReefSide = Field.getReefSideMiddle(reefSide, false).getRotation();

		Pose2d algaeRemovePoseBySide = Field
			.getPoseBySide(new Pose2d(algaeRemovePose, headingToReefSide), Field.isOnBlueSide(robotPose.getTranslation()));

		chassisSpeeds = AimAssistMath
			.getRotationAssistedSpeeds(chassisSpeeds, robotPose.getRotation(), algaeRemovePoseBySide.getRotation(), swerveConstants);
		return AimAssistMath.getObjectAssistedSpeeds(
			chassisSpeeds,
			robotPose,
			algaeRemovePoseBySide.getRotation(),
			algaeRemovePoseBySide.getTranslation(),
			swerveConstants,
			swerveState
		);
	}

	private ChassisSpeeds handleAlgaeIntakeAimAssist(
		ChassisSpeeds chassisSpeeds,
		Pose2d robotPose,
		Translation2d closestAlgae,
		SwerveState swerveState
	) {
		Rotation2d targetHeading = FieldMath.getRelativeTranslation(robotPose.getTranslation(), closestAlgaeSupplier.get().get()).getAngle();

		chassisSpeeds = AimAssistMath.getRotationAssistedSpeeds(chassisSpeeds, robotPose.getRotation(), targetHeading, swerveConstants);
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, targetHeading, closestAlgae, swerveConstants, swerveState);
	}

	private ChassisSpeeds handleAngleCageAssist(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading) {
		Rotation2d headingToCage = ScoringHelpers.getHeadingForCage();
		return AimAssistMath.getRotationAssistedSpeeds(chassisSpeeds, robotHeading, headingToCage, swerveConstants);
	}

	private ChassisSpeeds handleCageAimAssist(ChassisSpeeds chassisSpeeds, Pose2d robotPose, Cage cage, SwerveState swerveState) {
		Translation2d cageTranslation = Field.getCage(cage).getTranslation();
		Rotation2d headingToCage = Field.getCage(cage).getRotation();

		chassisSpeeds = AimAssistMath.getRotationAssistedSpeeds(chassisSpeeds, robotPose.getRotation(), headingToCage, swerveConstants);
		return AimAssistMath.getObjectAssistedSpeeds(chassisSpeeds, robotPose, headingToCage, cageTranslation, swerveConstants, swerveState);
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
