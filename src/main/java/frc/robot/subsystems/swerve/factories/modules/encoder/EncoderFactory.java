package frc.robot.subsystems.swerve.factories.modules.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.angleencoder.CANCoderEncoder;
import frc.robot.hardware.angleencoder.EmptyAngleEncoder;
import frc.robot.hardware.angleencoder.IAngleEncoder;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;

public class EncoderFactory {

	public static IAngleEncoder create(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		return switch (swerveName) {
			case SWERVE -> createSwerveEncoder(modulePosition);
		};
	}

	private static IAngleEncoder createSwerveEncoder(ModuleUtils.ModulePosition modulePosition) {
		String logPath = SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/Encoder/";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT -> new CANCoderEncoder(logPath, EncoderRealConstants.FRONT_LEFT.getCANcoderWithConfig(logPath));
				case FRONT_RIGHT -> new CANCoderEncoder(logPath, EncoderRealConstants.FRONT_RIGHT.getCANcoderWithConfig(logPath));
				case BACK_LEFT -> new CANCoderEncoder(logPath, EncoderRealConstants.BACK_LEFT.getCANcoderWithConfig(logPath));
				case BACK_RIGHT -> new CANCoderEncoder(logPath, EncoderRealConstants.BACK_RIGHT.getCANcoderWithConfig(logPath));
			};
			case SIMULATION -> new EmptyAngleEncoder(logPath);
		};
	}

	public static InputSignal<Rotation2d> createPositionSignal(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		return switch (swerveName) {
			case SWERVE -> createPositionSignal(modulePosition);
		};
	}

	private static InputSignal<Rotation2d> createPositionSignal(ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT -> EncoderRealConstants.FRONT_LEFT.getPositionSignal();
				case FRONT_RIGHT -> EncoderRealConstants.FRONT_RIGHT.getPositionSignal();
				case BACK_LEFT -> EncoderRealConstants.BACK_LEFT.getPositionSignal();
				case BACK_RIGHT -> EncoderRealConstants.BACK_RIGHT.getPositionSignal();
			};
			case SIMULATION -> null; // TODO
		};
	}

}
