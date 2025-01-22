package frc.robot.subsystems.arm.factory;

import frc.robot.subsystems.arm.Arm;

public class ArmFactory {

	public static Arm create(String logPath) {
		return KrakenX60ArmBuilder.build(logPath);
	}

}
