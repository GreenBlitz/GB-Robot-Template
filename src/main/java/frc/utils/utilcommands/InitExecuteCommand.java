package frc.utils.utilcommands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A command that runs a given runnable when it is initialized, and another runnable on execute.
 */
public class InitExecuteCommand extends FunctionalCommand {

    /**
     * Creates a new InitExecuteCommand. Will run the given runnables when the command starts and when on execute.
     *
     * @param onInit the runnable to run on command init
     * @param onExecute the runnable to run on command execute
     * @param requirements the subsystems required by this command
     */
    public InitExecuteCommand(Runnable onInit, Runnable onExecute, SubsystemBase... requirements) {
        super(onInit, onExecute, (interrupted) -> {}, () -> false, requirements);
    }

}
