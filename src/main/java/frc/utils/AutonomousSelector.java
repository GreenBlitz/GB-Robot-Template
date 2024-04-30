package frc.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class AutonomousSelector {

    private static final SendableChooser<Command> selector = buildSelector();

    public static SendableChooser<Command> buildSelector() {
        SendableChooser<Command> selector = AutoBuilder.buildAutoChooser();
        ShuffleboardTab tab = Shuffleboard.getTab("auto");
        tab.add("autonomous chooser", selector);
        return selector;
    }

    public static Command getChosenValue() {
        Logger.recordMetadata("autonomous chosen", selector.getSelected().getName());
        return selector.getSelected();
    }
}
