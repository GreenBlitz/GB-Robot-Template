package frc.utils.utilcommands;

import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.SubsystemBase;

/**
 * A command that runs a given runnable when it is initialized, and another runnable on execute.
 */
public class InitExecuteCommand extends FunctionalCommand {

	/**
	 * Creates a new InitExecuteCommand. Will run the given runnables when the command starts and when on execute.
	 *
	 * @param onInit       the runnable to run on command init
	 * @param onExecute    the runnable to run on command execute
	 * @param requirements the subsystems required by this command
	 */
	public InitExecuteCommand(Runnable onInit, Runnable onExecute, SubsystemBase... requirements) {
		super(onInit, onExecute, (interrupted) -> {}, () -> false, requirements);
	}

}
