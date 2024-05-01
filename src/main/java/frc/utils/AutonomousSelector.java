package frc.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;

public class AutonomousSelector {

    private static SendableChooser<Command> selector;

    public static void buildSelector() {
        selector = AutoBuilder.buildAutoChooser();
        ShuffleboardTab tab = Shuffleboard.getTab("auto");
        tab.add("autonomous selector", selector);
    }

    public static Command getChosenValue() {
        if (selector != null) {
            Logger.recordMetadata("autonomous chosen", selector.getSelected().getName());
            return selector.getSelected();
        }
        else {
            Logger.recordMetadata("autonomous chosen", "None");
            return new InstantCommand();
        }
    }
}
