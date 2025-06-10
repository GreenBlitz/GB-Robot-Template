package frc.robot.subsystems.algaeIntake.rollers.Factory;

import frc.robot.subsystems.algaeIntake.rollers.Rollers;
import frc.robot.subsystems.algaeIntake.rollers.RollersConstants;

public class RollersFactory {

	public static Rollers create(String logPath) {
		return RollersConstants.IS_MOTOR_SPARK_MAX ? SparkMaxRollersBuilder.generate(logPath) : TalonFXRollersBuilder.generate(logPath);
	}

}
