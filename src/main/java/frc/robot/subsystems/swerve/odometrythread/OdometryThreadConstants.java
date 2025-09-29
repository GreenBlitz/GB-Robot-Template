package frc.robot.subsystems.swerve.odometrythread;

import frc.robot.RobotConstants;

public class OdometryThreadConstants {

	public static final double SIMULATION_FREQUENCY_HERTZ = RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ;

	public static int getQueuesSize(double frequencyHertz, double mainThreadFrequencyHertz, int offset) {
		return (int) (frequencyHertz / mainThreadFrequencyHertz) + offset;
	}

}
