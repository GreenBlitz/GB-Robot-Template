package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.Robot;
import frc.robot.IDs;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.records.EncoderSignals;

public class EncoderFactory {

	public static IAngleEncoder createEncoder(String logPath, ModuleUtils.ModulePosition modulePosition) {
		logPath += ModuleConstants.MODULES_LOG_PATH_ADDITION + modulePosition + "/Encoder/";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT -> RealEncoderConstants.generateEncoder(logPath, IDs.CANCodersIDs.FRONT_LEFT_ENCODER);
				case FRONT_RIGHT -> RealEncoderConstants.generateEncoder(logPath, IDs.CANCodersIDs.FRONT_RIGHT_ENCODER);
				case BACK_LEFT -> RealEncoderConstants.generateEncoder(logPath, IDs.CANCodersIDs.BACK_LEFT_ENCODER);
				case BACK_RIGHT -> RealEncoderConstants.generateEncoder(logPath, IDs.CANCodersIDs.BACK_RIGHT_ENCODER);
			};
			case SIMULATION -> SimulationEncoderConstants.generateEncoder(logPath);
		};
	}

	public static EncoderSignals createSignals(IAngleEncoder angleEncoder) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealEncoderConstants.generateSignals((CANCoderEncoder) angleEncoder);
			case SIMULATION -> SimulationEncoderConstants.generateSignals();
		};
	}

}
