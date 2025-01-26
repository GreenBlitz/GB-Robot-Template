package frc.robot.subsystems.swerve.factories.modules.steer;

import frc.robot.IDs;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.records.SteerRequests;
import frc.robot.subsystems.swerve.module.records.SteerSignals;

public class SteerFactory {

	public static ControllableMotor createSteer(String logPath, ModuleUtils.ModulePosition modulePosition) {
		logPath += ModuleConstants.MODULES_LOG_PATH_ADDITION + "/" + modulePosition + "/Steer";
		return switch (modulePosition) {
			case FRONT_LEFT ->
				Falcon500SteerBuilder.buildSteer(logPath, IDs.TalonFXIDs.SWERVE_FRONT_LEFT_STEER, IDs.CANCodersIDs.SWERVE_FRONT_LEFT, true);
			case FRONT_RIGHT ->
				Falcon500SteerBuilder.buildSteer(logPath, IDs.TalonFXIDs.SWERVE_FRONT_RIGHT_STEER, IDs.CANCodersIDs.SWERVE_FRONT_RIGHT, true);
			case BACK_LEFT ->
				Falcon500SteerBuilder.buildSteer(logPath, IDs.TalonFXIDs.SWERVE_BACK_LEFT_STEER, IDs.CANCodersIDs.SWERVE_BACK_LEFT, false);
			case BACK_RIGHT ->
				Falcon500SteerBuilder.buildSteer(logPath, IDs.TalonFXIDs.SWERVE_BACK_RIGHT_STEER, IDs.CANCodersIDs.SWERVE_BACK_RIGHT, true);
		};
	}

	public static SteerRequests createRequests() {
		return Falcon500SteerBuilder.buildRequests();
	}

	public static SteerSignals createSignals(ControllableMotor steer) {
		return Falcon500SteerBuilder.buildSignals((TalonFXMotor) steer);
	}

}
