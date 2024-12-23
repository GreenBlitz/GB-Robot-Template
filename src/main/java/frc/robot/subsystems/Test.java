package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Test extends GBSubsystem{

    private String currentCommandName;

    public Test(String logPath) {
        super(logPath);
        currentCommandName = "No command is currently running on the subsystem";
    }

    @Override
    public String getCurrentCommandName() {
        return currentCommandName;
    }

    public Command logIDK(){
        currentCommandName = "logIDK";
        return new RunCommand(
                () -> Logger.recordOutput("yoooo", "IDK")
        );
    }

}
