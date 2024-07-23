package frc.robot.subsystems.swerve.swervecontainer;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.swervecontainer.real.RealSwerve;
import frc.robot.subsystems.swerve.swervecontainer.simulation.SimulationSwerve;

import static frc.robot.Robot.ROBOT_TYPE;

public class SwerveFactory {

    public static class SwerveContainerKey {

        private SwerveContainerKey(){}

    }

    private static final SwerveContainer REAL_SWERVE = new RealSwerve(new SwerveContainerKey());
    private static final SwerveContainer SIMULATION_SWERVE = new SimulationSwerve(new SwerveContainerKey());


    public static SwerveConstants createConstants() {
        return switch (ROBOT_TYPE) {
            case REAL, REPLAY -> REAL_SWERVE.getConstants();
            case SIMULATION -> SIMULATION_SWERVE.getConstants();
        };
    }




}
