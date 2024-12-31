package frc.robot.subsystems.swerve.factories.modules.steer;

import frc.robot.Robot;
import frc.robot.IDs;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.records.SteerRequests;
import frc.robot.subsystems.swerve.module.records.SteerSignals;

public class SteerFactory {

	public static ControllableMotor createSteer(String logPath, ModuleUtils.ModulePosition modulePosition) {
		logPath += ModuleConstants.MODULES_LOG_PATH_ADDITION + modulePosition + "/Steer/";
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

	public static SteerRequests createRequests() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> TalonFXSteerConstants.generateRequests();
		};
	}

	public static SteerSignals createSignals(ControllableMotor steer) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> TalonFXSteerConstants.generateSignals((TalonFXMotor) steer);
		};
	}

}
