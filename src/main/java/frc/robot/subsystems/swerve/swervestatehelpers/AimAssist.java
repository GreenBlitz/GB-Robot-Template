package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.utils.mirrorutils.MirrorableRotation2d;
import frc.utils.mirrorutils.MirrorableTranslation3d;

import java.util.function.Supplier;

public enum AimAssist {

    NONE(),

    //todo - NOTE(),

    //todo - STAGE();

    SPEAKER(FieldConstants.SPEAKER),

    AMP(FieldConstants.ANGLE_TO_AMP);


    public final Supplier<MirrorableRotation2d> targetAngleSupplier;

    AimAssist() {
        targetAngleSupplier = () -> MirrorableRotation2d.fromDegrees(0, false);
    }

    AimAssist(MirrorableRotation2d targetRotation) {
        this.targetAngleSupplier = () -> targetRotation;
    }

    AimAssist(MirrorableRotation2dSupplier targetAllianceRotation) {
        this.targetAngleSupplier = targetAllianceRotation;
    }

    AimAssist(MirrorableTranslation3d targetAllianceTranslation) {
        this.targetAngleSupplier = () -> getTargetAngleFromTargetTranslation(targetAllianceTranslation);
    }

    AimAssist(MirrorableTranslation3dSupplier targetAllianceTranslationSupplier) {
        this.targetAngleSupplier = () -> getTargetAngleFromTargetTranslation(targetAllianceTranslationSupplier.get());
    }

    //Todo - Maybe in kinda math util or pose util
    private interface MirrorableRotation2dSupplier extends Supplier<frc.utils.mirrorutils.MirrorableRotation2d> {}

    //Todo - Maybe in kinda math util or pose util
    private interface MirrorableTranslation3dSupplier extends Supplier<MirrorableTranslation3d> {}

    //Todo - Maybe in kinda math util or pose util or swerveMath
    private MirrorableRotation2d getTargetAngleFromTargetTranslation(MirrorableTranslation3d targetPose2d) {
        Pose2d currentBluePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose();
        Translation2d targetMirroredPose = targetPose2d.get().toTranslation2d();
        double wantedAngleRadians = Math.atan2(
                targetMirroredPose.getY() - currentBluePose.getY(),
                targetMirroredPose.getX() - currentBluePose.getX()
        );
        return MirrorableRotation2d.fromRadians(wantedAngleRadians, false);
    }
}
