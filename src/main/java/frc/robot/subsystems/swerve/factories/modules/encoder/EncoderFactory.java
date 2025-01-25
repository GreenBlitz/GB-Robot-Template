package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.records.EncoderSignals;

public class EncoderFactory {

	public static IAngleEncoder createEncoder(String logPath, ModuleUtils.ModulePosition modulePosition) {
		logPath += ModuleConstants.MODULES_LOG_PATH_ADDITION + "/" + modulePosition + "/Encoder";
		return switch (Robot.ROBOT_TYPE) {
//            case REAL -> switch (modulePosition) {
//                case FRONT_LEFT -> CANCoderEncoderBuilder.buildEncoder(logPath, IDs.CANCodersIDs.FRONT_LEFT_ENCODER);
//                case FRONT_RIGHT -> CANCoderEncoderBuilder.buildEncoder(logPath, IDs.CANCodersIDs.FRONT_RIGHT_ENCODER);
//                case BACK_LEFT -> CANCoderEncoderBuilder.buildEncoder(logPath, IDs.CANCodersIDs.BACK_LEFT_ENCODER);
//                case BACK_RIGHT -> CANCoderEncoderBuilder.buildEncoder(logPath, IDs.CANCodersIDs.BACK_RIGHT_ENCODER);
//            }; TODO
			case SIMULATION, REAL -> SimulationEncoderBuilder.buildEncoder(logPath);
		};
	}

	public static EncoderSignals createSignals(IAngleEncoder angleEncoder) {
		return switch (Robot.ROBOT_TYPE) {
//            case REAL -> CANCoderEncoderBuilder.buildSignals((CANCoderEncoder) angleEncoder); TODO
			case SIMULATION, REAL -> SimulationEncoderBuilder.buildSignals();
		};
	}

}
