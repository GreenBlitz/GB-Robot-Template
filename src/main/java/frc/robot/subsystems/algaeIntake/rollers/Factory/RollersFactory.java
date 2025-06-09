package frc.robot.subsystems.algaeIntake.rollers.Factory;

import frc.robot.subsystems.algaeIntake.rollers.Rollers;
import frc.robot.subsystems.algaeIntake.rollers.RollersConstants;

public class RollersFactory {

	public static Rollers create(String logPath) {
		if (RollersConstants.isMotorSparkMax) {
			return SparkMaxRollersBuilder.generate(logPath);
		} else {
			return TalonFXRollersBuilder.generate(logPath);
		}
	}

}
