package frc.utils.pathplannerutils;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class AutonomousChooser {

    private static final SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();

    public static void addSelectorToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");
        tab.add("Autonomous Chooser", chooser);
    }

    public static Command getChosenValue() {
        Logger.recordOutput("Autonomous Chosen", chooser.getSelected().getName());
        return chooser.getSelected();
    }
}
