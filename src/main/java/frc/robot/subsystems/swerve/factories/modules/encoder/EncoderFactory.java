package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.encoder.EmptyEncoder;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.robot.subsystems.swerve.modules.encoder.cancoder.CancoderEncoder;

public class EncoderFactory {

	public static IEncoder create(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		return switch (swerveName) {
			case SWERVE -> createSwerveEncoder(modulePosition);
		};
	}

	private static IEncoder createSwerveEncoder(ModuleUtils.ModulePosition modulePosition) {
		String logPathPrefix = SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT ->
					new CancoderEncoder(IDs.CANCodersIDs.FRONT_LEFT_ENCODER, EncoderRealConstants.generateEncoderConfig(), logPathPrefix);
				// @formatter:off
				case FRONT_RIGHT ->
						new CancoderEncoder(IDs.CANCodersIDs.FRONT_RIGHT_ENCODER, EncoderRealConstants.generateEncoderConfig(), logPathPrefix);
				// @formatter:on
				case BACK_LEFT ->
					new CancoderEncoder(IDs.CANCodersIDs.BACK_LEFT_ENCODER, EncoderRealConstants.generateEncoderConfig(), logPathPrefix);
				case BACK_RIGHT ->
					new CancoderEncoder(IDs.CANCodersIDs.BACK_RIGHT_ENCODER, EncoderRealConstants.generateEncoderConfig(), logPathPrefix);
			};
			case SIMULATION -> new EmptyEncoder();
		};
	}

}
