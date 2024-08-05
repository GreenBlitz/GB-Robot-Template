package frc.robot.subsystems.swerve.factories.modules.drive;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.modules.drive.replay.EmptyDrive;
import frc.robot.subsystems.swerve.modules.drive.simulation.SimulationDrive;
import frc.robot.subsystems.swerve.modules.drive.talonfx.TalonFXDrive;

public class DriveFactory {

    public static IDrive create(ModuleUtils.ModulePosition modulePosition, ModuleUtils.ModuleType moduleType) {
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> getRealSteer(modulePosition, moduleType);
            case SIMULATION -> new SimulationDrive(DriveSimulationConstants.getDriveConstants());
            case REPLAY -> new EmptyDrive();
        };
    }

    private static IDrive getRealSteer(ModuleUtils.ModulePosition modulePosition, ModuleUtils.ModuleType moduleType) {
        return switch (moduleType) {
            case TALON_FX -> switch (modulePosition) {
                case FRONT_LEFT -> new TalonFXDrive(DriveRealConstants.FRONT_LEFT_CONSTANTS);
                case FRONT_RIGHT -> new TalonFXDrive(DriveRealConstants.FRONT_RIGHT_CONSTANTS);
                case BACK_LEFT -> new TalonFXDrive(DriveRealConstants.BACK_LEFT_CONSTANTS);
                case BACK_RIGHT -> new TalonFXDrive(DriveRealConstants.BACK_RIGHT_CONSTANTS);
            };
        };
    }

}
