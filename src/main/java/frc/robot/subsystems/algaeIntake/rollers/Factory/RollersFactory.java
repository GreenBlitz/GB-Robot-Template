package frc.robot.subsystems.algaeIntake.rollers.Factory;

import frc.robot.subsystems.algaeIntake.rollers.Rollers;

public class RollersFactory {

	public static Rollers create(String logPath) {
		return TalonFXRollersBuilder.generate(logPath);
	}

}
