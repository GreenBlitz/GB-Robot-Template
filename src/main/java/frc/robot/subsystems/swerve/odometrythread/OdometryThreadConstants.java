package frc.robot.subsystems.swerve.odometrythread;

import frc.robot.RobotConstants;

public class OdometryThreadConstants {

	public static final double FREQUENCY_HERTZ = 250.0;

	public static final double SIMULATION_FREQUENCY_HERTZ = 50.0;

	public static final String NAME = "OdometryThread";

	public static final int MAX_VALUE_CAPACITY_PER_UPDATE = (int) (FREQUENCY_HERTZ / RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ) + 5;

	public static final int THREAD_PRIORITY = 2;

	public static final boolean IS_BUS_CHAIN_CANFD = false;

}
