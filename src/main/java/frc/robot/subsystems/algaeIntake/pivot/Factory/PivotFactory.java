package frc.robot.subsystems.algaeIntake.pivot.Factory;

import frc.robot.subsystems.algaeIntake.pivot.Pivot;

public class PivotFactory {

	public static Pivot create(String logPath) {
		return TalonFXPivotBuilder.generate(logPath);
	}

}
