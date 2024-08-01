package frc.robot.subsystems.swerve.factories.modules.steer;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.replay.EmptySteer;
import frc.robot.subsystems.swerve.modules.steer.simulation.SimulationSteer;
import frc.robot.subsystems.swerve.modules.steer.talonfx.TalonFXSteer;

public class SteerFactory {

    public static ISteer create(ModuleUtils.ModuleName moduleName) {
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> switch (moduleName){
                case FRONT_LEFT -> new TalonFXSteer(RealSteerConstants.FRONT_LEFT_CONSTANTS);
                case FRONT_RIGHT -> new TalonFXSteer(RealSteerConstants.FRONT_RIGHT_CONSTANTS);
                case BACK_LEFT -> new TalonFXSteer(RealSteerConstants.BACK_LEFT_CONSTANTS);
                case BACK_RIGHT -> new TalonFXSteer(RealSteerConstants.BACK_RIGHT_CONSTANTS);
            };
            case SIMULATION -> new SimulationSteer(SteerSimulationConstants.getSteerConstants());
            case REPLAY -> new EmptySteer();
        };
    }

}
