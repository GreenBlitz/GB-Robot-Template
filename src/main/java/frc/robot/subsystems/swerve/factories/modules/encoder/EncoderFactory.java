package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.records.EncoderSignals;

public class EncoderFactory {

	public static IAngleEncoder createEncoder(String logPath, ModuleUtil.ModulePosition modulePosition) {
		logPath += ModuleConstants.MODULES_LOG_PATH_ADDITION + "/" + modulePosition + "/Encoder";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT -> CANCoderEncoderBuilder.buildEncoder(logPath, IDs.CANCoderIDs.SWERVE_FRONT_LEFT);
				case FRONT_RIGHT -> CANCoderEncoderBuilder.buildEncoder(logPath, IDs.CANCoderIDs.SWERVE_FRONT_RIGHT);
				case BACK_LEFT -> CANCoderEncoderBuilder.buildEncoder(logPath, IDs.CANCoderIDs.SWERVE_BACK_LEFT);
				case BACK_RIGHT -> CANCoderEncoderBuilder.buildEncoder(logPath, IDs.CANCoderIDs.SWERVE_BACK_RIGHT);
			};
			case REPLAY, SIMULATION -> SimulationEncoderBuilder.buildEncoder(logPath);
		};
	}

	public static EncoderSignals createSignals(IAngleEncoder angleEncoder) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> CANCoderEncoderBuilder.buildSignals((CANCoderEncoder) angleEncoder);
			case SIMULATION, REPLAY -> SimulationEncoderBuilder.buildSignals();
		};
	}

}
