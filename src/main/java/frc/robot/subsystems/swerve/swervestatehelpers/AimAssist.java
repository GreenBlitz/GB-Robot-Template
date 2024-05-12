package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.utils.allianceutils.AllianceRotation2d;
import frc.utils.allianceutils.AllianceTranslation2d;

import java.util.function.Supplier;

public enum AimAssist {

    NONE(),

    //todo - NOTE(),

    //todo - STAGE();

    SPEAKER(FieldConstants::getAllianceSpeaker),

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
        this.targetAngleSupplier =
                () -> SwerveMath.getTargetAngleFromTargetTranslation(
                        RobotContainer.POSE_ESTIMATOR.getCurrentPose().getTranslation2d(),
                        targetAllianceTranslation
                );
    }

    AimAssist(AllianceTranslation2dSupplier targetAllianceTranslationSupplier) {
        this.targetAngleSupplier = () -> SwerveMath.getTargetAngleFromTargetTranslation(
                RobotContainer.POSE_ESTIMATOR.getCurrentPose().getTranslation2d(),
                targetAllianceTranslationSupplier.get()
        );
    }

    //Todo - Maybe in kinda math util or pose util
    private interface AllianceRotation2dSupplier extends Supplier<AllianceRotation2d> {}

    //Todo - Maybe in kinda math util or pose util
    private interface AllianceTranslation2dSupplier extends Supplier<AllianceTranslation2d> {}
}
