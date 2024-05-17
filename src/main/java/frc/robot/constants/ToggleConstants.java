package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ToggleConstants {

    private static final SendableChooser<DriverStation.Alliance> ALLIANCE_CHOOSER = new SendableChooser<>();
    static {
        ALLIANCE_CHOOSER.setDefaultOption("BLUE", DriverStation.Alliance.Blue);
        ALLIANCE_CHOOSER.addOption("RED", DriverStation.Alliance.Red);
    }

    public static final LoggedDashboardChooser<DriverStation.Alliance> SIMULATION_ALLIANCE =
            new LoggedDashboardChooser<>("Simulation Alliance", ALLIANCE_CHOOSER);


    private static final SendableChooser<Boolean> BATTERY_LIMITER_CHOOSER = new SendableChooser<>();
    static {
        BATTERY_LIMITER_CHOOSER.setDefaultOption("ENABLE", true);
        BATTERY_LIMITER_CHOOSER.addOption("DISABLE", false);
    }

    // Use this to disable battery limiter, don't comment!!!
    public static final LoggedDashboardChooser<Boolean> DISABLE_BATTERY_LIMITER =
            new LoggedDashboardChooser<>("Battery Limiter", BATTERY_LIMITER_CHOOSER);

}
