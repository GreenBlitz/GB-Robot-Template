package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.robot.subsystems.swerve.modules.encoder.cancoder.CancoderEncoder;
import frc.robot.subsystems.swerve.modules.encoder.simulation.EmptyEncoder;

public class EncoderFactory {

    public static IEncoder[] create(){
        return switch (Robot.ROBOT_TYPE){
            case REAL -> new CancoderEncoder[]{
                    new CancoderEncoder(IDs.CANCodersIDs.FRONT_LEFT_ENCODER, RealEncoderConstants.ENCODER_CONFIG),
                    new CancoderEncoder(IDs.CANCodersIDs.FRONT_RIGHT_ENCODER, RealEncoderConstants.ENCODER_CONFIG),
                    new CancoderEncoder(IDs.CANCodersIDs.BACK_LEFT_ENCODER, RealEncoderConstants.ENCODER_CONFIG),
                    new CancoderEncoder(IDs.CANCodersIDs.BACK_RIGHT_ENCODER, RealEncoderConstants.ENCODER_CONFIG)
            };
            case SIMULATION, REPLAY -> new EmptyEncoder[]{
                    new EmptyEncoder(),
                    new EmptyEncoder(),
                    new EmptyEncoder(),
                    new EmptyEncoder()
            };
        };
    }

}
