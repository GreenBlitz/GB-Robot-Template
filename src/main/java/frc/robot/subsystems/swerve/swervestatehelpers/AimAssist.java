package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Function;

public enum AimAssist {

    NONE(0),
    SPEAKER(
            4,
            (robotPose -> new Rotation2d()),
            (robotPose -> new Translation2d())
    ),
    AMP(
            4,
            (robotPose -> Rotation2d.fromDegrees(90)),
            (robotPose -> new Translation2d())
    );

    public final int actionIndicator;
    public final Function<Pose2d, Rotation2d> targetRotationSupplier;
    public final Function<Pose2d, Translation2d> targetTranslationSupplier;

    AimAssist(int actionIndicator) {
        this(actionIndicator, (robotPose) -> null, (robotPose) -> null);
    }

    AimAssist(int actionIndicator, Function<Pose2d, Rotation2d> targetRotationSupplier,
            Function<Pose2d, Translation2d> targetTranslationSupplier) {
        this.actionIndicator = actionIndicator;
        this.targetRotationSupplier = targetRotationSupplier;
        this.targetTranslationSupplier = targetTranslationSupplier;
    }
}
