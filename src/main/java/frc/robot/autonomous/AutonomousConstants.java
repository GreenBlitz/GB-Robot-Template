package frc.robot.autonomous;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;


public class AutonomousConstants {

	public static final double PATHFIND_OR_FOLLOW_PATH_TOLERANCE_METERS = 0.2;

	public static final Supplier<Command> INTAKE_COMMAND = () -> NamedCommands.getCommand("wait");

	public static final Supplier<Command> BEFORE_SHOOTING_COMMAND = () -> NamedCommands.getCommand("they don't love you");

	public static final Supplier<Command> SHOOTING_COMMAND = () -> NamedCommands.getCommand("like I love you");

}
