package frc.utils.utilcommands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A command that runs a given runnable on execute, and another runnable when it ends.
 */
public class ExecuteEndCommand extends FunctionalCommand {

    /**
     * Creates a new ExecuteEndCommand. Will run the given runnables when the command on execute and when it ends.
     *
     * @param onExecute the runnable to run on command execute
     * @param onEnd the runnable to run on command end
     * @param requirements the subsystems required by this command
     */
    public ExecuteEndCommand(Runnable onExecute, Runnable onEnd, SubsystemBase... requirements) {
        super(() -> {}, onExecute, (interrupted) -> onEnd.run(), () -> false, requirements);
    }

}
