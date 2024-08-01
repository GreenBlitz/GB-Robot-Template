package frc.robot.subsystems.swerve.factories.modules.steer;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.replay.EmptySteer;
import frc.robot.subsystems.swerve.modules.steer.simulation.SimulationSteer;
import frc.robot.subsystems.swerve.modules.steer.talonfx.TalonFXSteer;

public class SteerFactory {

    public static ISteer[] create() {
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> new TalonFXSteer[]{
                    new TalonFXSteer(RealSteerConstants.STEER_CONSTANTS[0]),
                    new TalonFXSteer(RealSteerConstants.STEER_CONSTANTS[1]),
                    new TalonFXSteer(RealSteerConstants.STEER_CONSTANTS[2]),
                    new TalonFXSteer(RealSteerConstants.STEER_CONSTANTS[3])
            };
            case SIMULATION -> new SimulationSteer[]{
                    new SimulationSteer(SteerSimulationConstants.getSteerConstants()),
                    new SimulationSteer(SteerSimulationConstants.getSteerConstants()),
                    new SimulationSteer(SteerSimulationConstants.getSteerConstants()),
                    new SimulationSteer(SteerSimulationConstants.getSteerConstants())
            };
            case REPLAY -> new EmptySteer[]{
                    new EmptySteer(),
                    new EmptySteer(),
                    new EmptySteer(),
                    new EmptySteer()
            };
        };
    }

}
