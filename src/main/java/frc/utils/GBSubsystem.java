package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GBSubsystem extends SubsystemBase {

    @Override
    public void periodic() {
        super.periodic();
        Logger.recordOutput("subsystems/" + getClass().getSimpleName(), getCurrentCommandName());
    }

    private String getCurrentCommandName() {
        final Command currentCommand = getCurrentCommand();
        return currentCommand != null ? currentCommand.getName() : "no command is currently running on the subsystem";
    }

}
