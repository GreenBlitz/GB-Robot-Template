package frc.robot.subsystems.swerve.factories.modules.steer;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.stuffs.SteerStuff;

public class SteerFactory {

	private static SteerStuff createSwerveSteer(String logPath, ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT ->
					SteerRealConstants
						.generateSteerStuff(logPath, IDs.TalonFXIDs.FRONT_LEFT_STEER_MOTOR, IDs.CANCodersIDs.FRONT_LEFT_ENCODER, true);
				case FRONT_RIGHT ->
					SteerRealConstants
						.generateSteerStuff(logPath, IDs.TalonFXIDs.FRONT_RIGHT_STEER_MOTOR, IDs.CANCodersIDs.FRONT_RIGHT_ENCODER, true);
				case BACK_LEFT ->
					SteerRealConstants
						.generateSteerStuff(logPath, IDs.TalonFXIDs.BACK_LEFT_STEER_MOTOR, IDs.CANCodersIDs.BACK_LEFT_ENCODER, false);
				case BACK_RIGHT ->
					SteerRealConstants
						.generateSteerStuff(logPath, IDs.TalonFXIDs.BACK_RIGHT_STEER_MOTOR, IDs.CANCodersIDs.BACK_RIGHT_ENCODER, true);
			};
			case SIMULATION -> null;// TODO
		};
	}

	public static SteerStuff create(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		String logPath = SwerveType.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/Steer/";
		return switch (swerveType) {
			case SWERVE -> createSwerveSteer(logPath, modulePosition);
		};
	}

}
