package frc.utils.pathplannerutils;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousChooser {

    private final LoggedDashboardChooser<Command> chooser;

    public AutonomousChooser(String name, String defaultOption) {
        this.chooser = new LoggedDashboardChooser<Command>(name, AutoBuilder.buildAutoChooser(defaultOption));
    }

    public AutonomousChooser(String name) {
        this(name, "");
    }

    public Command getChosenValue() {
        return chooser.get();
    }
}
