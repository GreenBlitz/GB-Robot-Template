package frc.robot.subsystems.swerve.factories.modules.drive;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.modules.drive.replay.EmptyDrive;
import frc.robot.subsystems.swerve.modules.drive.simulation.SimulationDrive;
import frc.robot.subsystems.swerve.modules.drive.talonfx.TalonFXDrive;

public class DriveFactory {

    public static IDrive[] create() {
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> new TalonFXDrive[]{
                    new TalonFXDrive(DriveRealConstants.DRIVE_CONSTANTS[0]),
                    new TalonFXDrive(DriveRealConstants.DRIVE_CONSTANTS[1]),
                    new TalonFXDrive(DriveRealConstants.DRIVE_CONSTANTS[2]),
                    new TalonFXDrive(DriveRealConstants.DRIVE_CONSTANTS[3]),
            };
            case SIMULATION -> new SimulationDrive[]{
                    new SimulationDrive(DriveSimulationConstants.getDriveConstants()),
                    new SimulationDrive(DriveSimulationConstants.getDriveConstants()),
                    new SimulationDrive(DriveSimulationConstants.getDriveConstants()),
                    new SimulationDrive(DriveSimulationConstants.getDriveConstants())
            };
            case REPLAY -> new IDrive[]{
                    new EmptyDrive(),
                    new EmptyDrive(),
                    new EmptyDrive(),
                    new EmptyDrive()
            };
        };
    }

}
