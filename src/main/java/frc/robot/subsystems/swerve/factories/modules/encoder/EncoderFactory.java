package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.stuffs.EncoderStuff;

public class EncoderFactory {

	private static EncoderStuff createSwerveEncoder(String logPath, ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT -> EncoderRealConstants.generateEncoderStuff(logPath, IDs.CANCodersIDs.FRONT_LEFT_ENCODER);
				case FRONT_RIGHT -> EncoderRealConstants.generateEncoderStuff(logPath, IDs.CANCodersIDs.FRONT_RIGHT_ENCODER);
				case BACK_LEFT -> EncoderRealConstants.generateEncoderStuff(logPath, IDs.CANCodersIDs.BACK_LEFT_ENCODER);
				case BACK_RIGHT -> EncoderRealConstants.generateEncoderStuff(logPath, IDs.CANCodersIDs.BACK_RIGHT_ENCODER);
			};
			case SIMULATION -> null;// TODO
		};
	}

	public static EncoderStuff create(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		String logPath = swerveName.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/Encoder/";
		return switch (swerveName) {
			case SWERVE -> createSwerveEncoder(logPath, modulePosition);
		};
	}

}
