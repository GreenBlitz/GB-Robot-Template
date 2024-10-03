package frc.robot.subsystems.wrist.facotry;

import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristStuff;

public class WristFactory {
	
	public static WristStuff generateWristStuff(String logPath){
		return switch (Robot.ROBOT_TYPE){
			case REAL -> RealWristConstants.generateWristStuff(logPath);
			case SIMULATION -> null;
		};
	}
	
}
