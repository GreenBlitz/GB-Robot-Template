package frc.robot.subsystems.swerve.factories.modules.steer;

import frc.robot.IDs;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.records.SteerRequests;
import frc.robot.subsystems.swerve.module.records.SteerSignals;

public class SteerFactory {

	public static ControllableMotor createSteer(String logPath, ModuleUtil.ModulePosition modulePosition) {
		logPath += ModuleConstants.MODULES_LOG_PATH_ADDITION + "/" + modulePosition + "/Steer";
		return switch (modulePosition) {
			case FRONT_LEFT ->
				KrakenX60SteerBuilder.buildSteer(logPath, IDs.TalonFXIDs.SWERVE_FRONT_LEFT_STEER, IDs.CANCoderIDs.SWERVE_FRONT_LEFT, false);
			case FRONT_RIGHT ->
				KrakenX60SteerBuilder.buildSteer(logPath, IDs.TalonFXIDs.SWERVE_FRONT_RIGHT_STEER, IDs.CANCoderIDs.SWERVE_FRONT_RIGHT, false);
			case BACK_LEFT ->
				KrakenX60SteerBuilder.buildSteer(logPath, IDs.TalonFXIDs.SWERVE_BACK_LEFT_STEER, IDs.CANCoderIDs.SWERVE_BACK_LEFT, false);
			case BACK_RIGHT ->
				KrakenX60SteerBuilder.buildSteer(logPath, IDs.TalonFXIDs.SWERVE_BACK_RIGHT_STEER, IDs.CANCoderIDs.SWERVE_BACK_RIGHT, false);
		};
	}

	public static SteerRequests createRequests() {
		return KrakenX60SteerBuilder.buildRequests();
	}

	public static SteerSignals createSignals(ControllableMotor steer) {
		return KrakenX60SteerBuilder.buildSignals((TalonFXMotor) steer);
	}

}
