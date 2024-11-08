package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.stuffs.EncoderStuff;

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

	public static EncoderStuff create(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		String logPath = swerveType.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/Encoder/";
		return switch (swerveType) {
			case SWERVE -> createSwerveEncoder(logPath, modulePosition);
		};
	}

}
