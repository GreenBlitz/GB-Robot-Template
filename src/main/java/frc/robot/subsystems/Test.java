package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class Test extends GBSubsystem{

    public Test(String logPath) {
        super(logPath, () ->);
    }

    private String getCurrentCommandName() {
        Command currentCommand = getCurrentCommand();
        return currentCommand != null ? currentCommand.getName() : "no command is currently running on the subsystem";
    }
}
