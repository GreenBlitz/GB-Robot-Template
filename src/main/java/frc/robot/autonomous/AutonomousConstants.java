package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;

import java.util.function.Function;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous/";

	public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

	public static final double CLOSE_TO_TARGET_POSITION_DEADBAND_METERS = 0.5;

	public static final Function<Robot, Command> INTAKE_COMMAND = robot -> robot.getSuperStructure().setState(RobotState.INTAKE);

	public static final Function<Robot, Command> BEFORE_SHOOTING_COMMAND = robot -> robot.getSuperStructure().setState(RobotState.PRE_SPEAKER);

	public static final Function<Robot, Command> SHOOTING_COMMAND = robot -> robot.getSuperStructure().setState(RobotState.SPEKER);

}
