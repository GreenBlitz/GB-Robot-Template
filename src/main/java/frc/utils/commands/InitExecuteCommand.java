package frc.utils.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.utils.GBSubsystem;

public class InitExecuteCommand extends FunctionalCommand {

    /**
     * Creates a new InitExecuteCommand. Will run the given runnables when the command starts and when it
     * ends.
     *
     * @param onInit       the runnable to run on command init
     * @param onExecute    the runnable to run on command execute
     * @param requirements the subsystems required by this command
     */
    public InitExecuteCommand(Runnable onInit, Runnable onExecute, GBSubsystem... requirements) {
        super(onInit, onExecute, (interrupted) -> {}, () -> false, requirements);
    }
}
