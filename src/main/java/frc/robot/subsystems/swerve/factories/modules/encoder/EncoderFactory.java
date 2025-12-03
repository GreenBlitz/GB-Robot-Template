package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.records.EncoderSignals;

public class EncoderFactory {

	public static IAngleEncoder createEncoder(String logPath, ModuleUtil.ModulePosition modulePosition) {
		logPath += ModuleConstants.MODULES_LOG_PATH_ADDITION + "/" + modulePosition + "/Encoder";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> null;
			case SIMULATION -> SimulationEncoderBuilder.buildEncoder(logPath);
			case REPLAY -> null;
		};
	}

	public static EncoderSignals createSignals(IAngleEncoder angleEncoder) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> null;
			case SIMULATION -> SimulationEncoderBuilder.buildSignals();
			case REPLAY -> null;
		};
	}

}
