package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.utils.allianceutils.AllianceRotation2d;
import frc.utils.allianceutils.AllianceTranslation2d;

import java.util.function.Supplier;

public enum AimAssist {

    NONE(),

    //todo - NOTE(),

    //todo - STAGE();

    SPEAKER(FieldConstants.getAllianceSpeaker()),

    AMP(FieldConstants.ANGLE_TO_AMP);


    public final Supplier<AllianceRotation2d> targetAngleSupplier;

    AimAssist() {
        targetAngleSupplier = () -> AllianceRotation2d.fromBlueAllianceRotation(new Rotation2d());
    }

    AimAssist(AllianceRotation2d targetAllianceRotation) {
        this.targetAngleSupplier = () -> targetAllianceRotation;
    }

    AimAssist(AllianceRotation2dSupplier targetAllianceRotation) {
        this.targetAngleSupplier = targetAllianceRotation;
    }

    AimAssist(AllianceTranslation2d targetAllianceTranslation) {
        this.targetAngleSupplier = () -> getTargetAngleFromTargetTranslation(targetAllianceTranslation);
    }

    AimAssist(AllianceTranslation2dSupplier targetAllianceTranslationSupplier) {
        this.targetAngleSupplier = () -> getTargetAngleFromTargetTranslation(targetAllianceTranslationSupplier.get());
    }

    //Todo - Maybe in kinda math util or pose util
    private interface AllianceRotation2dSupplier extends Supplier<AllianceRotation2d> {}

    //Todo - Maybe in kinda math util or pose util
    private interface AllianceTranslation2dSupplier extends Supplier<AllianceTranslation2d> {}

    //Todo - Maybe in kinda math util or pose util or swerveMath
    private AllianceRotation2d getTargetAngleFromTargetTranslation(AllianceTranslation2d targetTranslation) {
        Pose2d currentBluePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getBlueAlliancePose();
        Rotation2d wantedAngle = Rotation2d.fromRadians(
                Math.atan2(
                        targetTranslation.getBlueAllianceTranslation2d().getY() - currentBluePose.getY(),
                        targetTranslation.getBlueAllianceTranslation2d().getX() - currentBluePose.getX()
                )
        );
        return AllianceRotation2d.fromBlueAllianceRotation(wantedAngle);
    }
}
