package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Swerve;

public enum DriveMode {

    FIELD_RELATIVE(Swerve::fieldRelativeSpeedsToSelfRelativeSpeeds),//todo - math class
    SELF_RELATIVE((speeds) -> speeds);


    private final DriveFunction driveFunction;

    DriveMode(DriveFunction driveFunction) {
        this.driveFunction = driveFunction;
    }

    public ChassisSpeeds getDriveModeRelativeChassisSpeeds(ChassisSpeeds speeds) {
        return driveFunction.getDriveModeRelativeChassisSpeeds(speeds);
    }

    private interface DriveFunction {

        ChassisSpeeds getDriveModeRelativeChassisSpeeds(ChassisSpeeds chassisSpeeds);

    }
}
