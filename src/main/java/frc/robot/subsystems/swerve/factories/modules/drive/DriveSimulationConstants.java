package frc.robot.subsystems.swerve.factories.modules.drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.swerve.modules.drive.simulation.SimulationDriveConstants;

class DriveSimulationConstants {

    private static final double DRIVE_GEAR_RATIO = 6.12;

    private static final double DRIVE_MOMENT_OF_INERTIA = 0.001;

    private static final boolean ENABLE_FOC_DRIVE = true;

    protected static SimulationDriveConstants getDriveConstants(){
        return new SimulationDriveConstants(
                new DCMotorSim(DCMotor.getFalcon500Foc(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
                ENABLE_FOC_DRIVE
        );
    }

}
