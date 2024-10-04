package frc.robot.subsystems.funnel.factory;

import frc.robot.Robot;
import frc.robot.subsystems.funnel.FunnelStuff;

public class FunnelFactory {

	public static FunnelStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealFunnelConstants.generateFunnelStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
