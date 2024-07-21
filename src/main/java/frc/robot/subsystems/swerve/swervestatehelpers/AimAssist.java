package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;

import java.util.function.Supplier;

public enum AimAssist {

    NONE(),

    SPEAKER(FieldConstants::getSpeaker),

    AMP(FieldConstants::getAngleToAmp);


    public final Supplier<Rotation2d> targetAngleSupplier;

    AimAssist() {
        targetAngleSupplier = () -> Rotation2d.fromDegrees(0);
    }

    AimAssist(Rotation2d targetRotation) {
        this.targetAngleSupplier = () -> targetRotation;
    }

    AimAssist(Rotation2dSupplier targetAllianceRotation) {
        this.targetAngleSupplier = targetAllianceRotation;
    }

    AimAssist(Translation3d targetAllianceTranslation) {
        this.targetAngleSupplier = () -> getTargetAngleFromTargetTranslation(targetAllianceTranslation);
    }

    AimAssist(Translation3dSupplier targetAllianceTranslationSupplier) {
        this.targetAngleSupplier = () -> getTargetAngleFromTargetTranslation(targetAllianceTranslationSupplier.get());
    }

    private interface Rotation2dSupplier extends Supplier<Rotation2d> {}

    private interface Translation3dSupplier extends Supplier<Translation3d> {}

    private Rotation2d getTargetAngleFromTargetTranslation(Translation3d targetPose2d) {
        Pose2d currentBluePose = Robot.poseEstimator.getCurrentPose();
        Translation2d targetBluePose = targetPose2d.toTranslation2d();
        double wantedAngleRadians = Math.atan2(
                targetBluePose.getY() - currentBluePose.getY(),
                targetBluePose.getX() - currentBluePose.getX()
        );
        return Rotation2d.fromRadians(wantedAngleRadians);
    }
}
