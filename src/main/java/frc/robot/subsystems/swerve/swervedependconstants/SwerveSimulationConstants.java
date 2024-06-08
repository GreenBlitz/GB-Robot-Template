package frc.robot.subsystems.swerve.swervedependconstants;

public class SwerveSimulationConstants implements SwerveDependConstants {

    private static final double SIMULATION_TIME_STEP_DISCRETION_FACTOR = 4;

    @Override
    public double getDiscretionFactor() {
        return SIMULATION_TIME_STEP_DISCRETION_FACTOR;
    }

}
