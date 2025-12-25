package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.MathConstants;
import frc.robot.statemachine.ScoringHelpers;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.subsystems.swerve.states.aimassist.AimAssistMath;
import frc.utils.alerts.Alert;

import java.util.Optional;
import java.util.function.Supplier;

public class SwerveStateHandler {

	private final Swerve swerve;
	private final SwerveConstants swerveConstants;
	private Optional<Supplier<Pose2d>> robotPoseSupplier;
	private Optional<Supplier<Boolean>> isTurretMoveLegalSupplier;

	public SwerveStateHandler(Swerve swerve) {
		this.swerve = swerve;
		this.swerveConstants = swerve.getConstants();
		this.robotPoseSupplier = Optional.empty();
		this.isTurretMoveLegalSupplier = Optional.empty();
	}

	public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
		this.robotPoseSupplier = Optional.of(robotPoseSupplier);
	}

	public void setIsTurretMoveLegalSupplier(Supplier<Boolean> isTurretMoveLegalSupplier) {
		this.isTurretMoveLegalSupplier = Optional.of(isTurretMoveLegalSupplier);
	}

	public ChassisSpeeds applyAimAssistOnChassisSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getAimAssist() == AimAssist.NONE) {
			return speeds;
		}
		if (robotPoseSupplier.isEmpty()) {
			reportMissingSupplier("robot pose");
			return speeds;
		}
		if (swerveState.getAimAssist() == AimAssist.LOOK_AT_TOWER) {
			if (isTurretMoveLegalSupplier.isEmpty()) {
				reportMissingSupplier("is turret move legal");
				return speeds;
			}
			if (isTurretMoveLegalSupplier.get().get() == false) {
				return handleLookAtTowerAimAssist(speeds);
			}
		}
		return speeds;
	}

	private ChassisSpeeds handleLookAtTowerAimAssist(ChassisSpeeds speeds) {
		Pose2d robotPose = robotPoseSupplier.get().get();
		Pose2d towerPose = ScoringHelpers.getClosestTower(robotPose).getPose();

		double dY = robotPose.getY() - towerPose.getY();
		double dX = robotPose.getX() - towerPose.getX();

		Rotation2d targetHeading = Rotation2d.fromRadians(Math.atan2(dY, dX));

		return AimAssistMath.getRotationAssistedSpeeds(speeds, robotPose.getRotation(), targetHeading, swerveConstants);
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

	private void reportMissingSupplier(String supplierName) {
		new Alert(Alert.AlertType.WARNING, swerve.getLogPath() + "/AimAssist/missing" + supplierName + " supplier").report();
	}

	public RotateAxis getFarRightRotateAxis() {
		return getFarRotateAxis(false);
	}

	public RotateAxis getFarLeftRotateAxis() {
		return getFarRotateAxis(true);
	}

}
