package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.constants.Field;

import java.util.function.Supplier;

public enum AimAssist {

	NONE(),

	SPEAKER(Field::getSpeaker),

	AMP(Field::getAngleToAmp);


	public final Supplier<Rotation2d> targetHeadingSupplier;

	AimAssist() {
		targetHeadingSupplier = () -> Rotation2d.fromDegrees(0);
	}

	AimAssist(Rotation2d targetRotation) {
		this.targetHeadingSupplier = () -> targetRotation;
	}

	AimAssist(Rotation2dSupplier targetAllianceRotation) {
		this.targetHeadingSupplier = targetAllianceRotation;
	}

	AimAssist(Translation3d targetAllianceTranslation) {
		this.targetHeadingSupplier = () -> getTargetHeadingFromTargetTranslation(targetAllianceTranslation);
	}

	AimAssist(Translation3dSupplier targetAllianceTranslationSupplier) {
		this.targetHeadingSupplier = () -> getTargetHeadingFromTargetTranslation(targetAllianceTranslationSupplier.get());
	}

	private interface Rotation2dSupplier extends Supplier<Rotation2d> {
	}

	private interface Translation3dSupplier extends Supplier<Translation3d> {
	}

	private Rotation2d getTargetHeadingFromTargetTranslation(Translation3d targetPose2d) {
		Pose2d currentBluePose = Robot.poseEstimator.getCurrentPose();
		Translation2d targetBluePose = targetPose2d.toTranslation2d();
		double wantedHeadingRadians = Math.atan2(targetBluePose.getY() - currentBluePose.getY(), targetBluePose.getX() - currentBluePose.getX());
		return Rotation2d.fromRadians(wantedHeadingRadians);
	}

}
