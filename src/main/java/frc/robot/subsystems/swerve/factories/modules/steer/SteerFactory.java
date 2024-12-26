package frc.robot.subsystems.swerve.factories.modules.steer;

import frc.robot.Robot;
import frc.robot.IDs;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.phoenix6.motor.TalonFXMotor;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.records.SteerRequests;
import frc.robot.subsystems.swerve.module.records.SteerSignals;

public class SteerFactory {

	private static ControllableMotor createSwerveSteer(String logPath, ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> switch (modulePosition) {
				case FRONT_LEFT ->
					TalonFXSteerConstants
						.generateSteer(logPath, IDs.TalonFXIDs.FRONT_LEFT_STEER_MOTOR, IDs.CANCodersIDs.FRONT_LEFT_ENCODER, true);
				case FRONT_RIGHT ->
					TalonFXSteerConstants
						.generateSteer(logPath, IDs.TalonFXIDs.FRONT_RIGHT_STEER_MOTOR, IDs.CANCodersIDs.FRONT_RIGHT_ENCODER, true);
				case BACK_LEFT ->
					TalonFXSteerConstants
						.generateSteer(logPath, IDs.TalonFXIDs.BACK_LEFT_STEER_MOTOR, IDs.CANCodersIDs.BACK_LEFT_ENCODER, false);
				case BACK_RIGHT ->
					TalonFXSteerConstants
						.generateSteer(logPath, IDs.TalonFXIDs.BACK_RIGHT_STEER_MOTOR, IDs.CANCodersIDs.BACK_RIGHT_ENCODER, true);
			};
		};
	}

	public static ControllableMotor createSteer(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		String logPath = SwerveType.SWERVE.getLogPath() + ModuleConstants.MODULES_LOG_PATH_ADDITION + modulePosition + "/Steer/";
		return switch (swerveType) {
			case SWERVE -> createSwerveSteer(logPath, modulePosition);
		};
	}

	private static SteerRequests createSteerRequests() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> TalonFXSteerConstants.generateRequests();
		};
	}

	public static SteerRequests createRequests(SwerveType swerveType) {
		return switch (swerveType) {
			case SWERVE -> createSteerRequests();
		};
	}

	private static SteerSignals createSteerSignals(ControllableMotor steer) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> TalonFXSteerConstants.generateSignals((TalonFXMotor) steer);
		};
	}

	public static SteerSignals createSignals(SwerveType swerveType, ControllableMotor steer) {
		return switch (swerveType) {
			case SWERVE -> createSteerSignals(steer);
		};
	}

}
