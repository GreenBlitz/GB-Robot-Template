package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GBSubsystem extends SubsystemBase {

    protected abstract String getLogPath();

    @Override
    public void periodic() {
        Logger.recordOutput(getLogPath() + "Current Command", getCurrentCommandName());
    }

    private String getCurrentCommandName() {
        Command currentCommand = getCurrentCommand();
        return currentCommand != null ? currentCommand.getName() : "no command is currently running on the subsystem";
    }

}
