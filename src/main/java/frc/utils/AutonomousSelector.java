package frc.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class AutonomousSelector {

    private final SendableChooser<Command> chooser;

    public AutonomousSelector() {
        chooser = AutoBuilder.buildAutoChooser();
        ShuffleboardTab tab = Shuffleboard.getTab("auto");
        tab.add("autonomous chooser", chooser);
    }

    public Command getChosenValue() {
        Logger.recordMetadata("autonomous chosen", chooser.getSelected().getName());
        return chooser.getSelected();
    }
}
