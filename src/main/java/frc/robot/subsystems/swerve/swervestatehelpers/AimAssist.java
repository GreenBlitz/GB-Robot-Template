package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

import java.util.function.Function;

public enum AimAssist {

    NONE(0),
    SPEAKER(4),
    NOTE (4),

    AMP(4);

    public final int actionIndicator;

    AimAssist(int actionIndicator) {
        this(actionIndicator, (robotPose) -> null, (robotPose) -> null);
    }

    AimAssist(int actionIndicator, Function<Pose2d, Rotation2d> targetRotationSupplier,
            Function<Pose2d, Translation2d> targetTranslationSupplier) {
        this.actionIndicator = actionIndicator;
    }
}
