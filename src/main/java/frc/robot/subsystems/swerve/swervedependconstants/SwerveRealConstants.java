package frc.robot.subsystems.swerve.swervedependconstants;

public class SwerveRealConstants implements SwerveDependConstants {

    private static final double REAL_TIME_STEP_DISCRETION_FACTOR = 8;

    @Override
    public double getDiscretionFactor() {
        return REAL_TIME_STEP_DISCRETION_FACTOR;
    }

}
