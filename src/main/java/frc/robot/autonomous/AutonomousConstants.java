package frc.robot.autonomous;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.Supplier;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous/";

	public static final double PATHFIND_OR_FOLLOW_PATH_TOLERANCE_METERS = 0.2;

	public static final Supplier<Command> INTAKE_COMMAND = () -> new WaitCommand(3);

	public static final Supplier<Command> BEFORE_SHOOTING_COMMAND = () -> new WaitCommand(1);

	public static final Supplier<Command> SHOOTING_COMMAND = () -> new WaitCommand(1.5);

}
