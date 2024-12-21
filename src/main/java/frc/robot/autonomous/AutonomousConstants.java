package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;

import java.util.function.Function;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous/";

	public static final double PATHFIND_OR_FOLLOW_PATH_TOLERANCE_METERS = 0.2;

	public static final Function<Robot, Command> INTAKE_COMMAND = robot -> robot.getSuperStructure().setState(RobotState.INTAKE);

	public static final Function<Robot, Command> BEFORE_SHOOTING_COMMAND = robot -> robot.getSuperStructure().setState(RobotState.PRE_SPEAKER);

	public static final Function<Robot, Command> SHOOTING_COMMAND = robot -> robot.getSuperStructure().setState(RobotState.SPEKER);

}
