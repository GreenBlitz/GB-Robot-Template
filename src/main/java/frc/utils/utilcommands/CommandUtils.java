package frc.utils.utilcommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.BooleanSupplier;

public class CommandUtils {

	public static Command dynamicChooseBetweenTwoCommands(
		BooleanSupplier firstCommandCondition,
		BooleanSupplier secondCommandCondition,
		Command firstCommand,
		Command secondCommand
	) {
		return new RepeatCommand(
			new SequentialCommandGroup(firstCommand.until(secondCommandCondition), secondCommand.until(firstCommandCondition))
		);
	}

	public static Command dynamicChooseBetweenTwoCommands(BooleanSupplier commandCondition, Command onTrue, Command onFalse) {
		return dynamicChooseBetweenTwoCommands(commandCondition, () -> !commandCondition.getAsBoolean(), onTrue, onFalse);
	}

}
