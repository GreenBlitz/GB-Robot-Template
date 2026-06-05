package frc.utils.utilcommands;

import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.SubsystemBase;

/**
 * A command that runs a given runnable on execute, and another runnable when it ends.
 */
public class ExecuteEndCommand extends FunctionalCommand {

	/**
	 * Creates a new ExecuteEndCommand. Will run the given runnables when the command on execute and when it ends.
	 *
	 * @param onExecute    the runnable to run on command execute
	 * @param onEnd        the runnable to run on command end
	 * @param requirements the subsystems required by this command
	 */
	public ExecuteEndCommand(Runnable onExecute, Runnable onEnd, SubsystemBase... requirements) {
		super(() -> {}, onExecute, (interrupted) -> onEnd.run(), () -> false, requirements);
	}

}
