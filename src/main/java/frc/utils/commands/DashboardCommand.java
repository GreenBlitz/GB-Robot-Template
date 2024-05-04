package frc.utils.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.GBSubsystem;
import frc.utils.DriverStationUtils;

import java.util.function.Consumer;

/**
 * This Command run a consumer through dashboard value
 */
public class DashboardCommand extends InitExecuteCommand {

    public DashboardCommand(
            double startingValue,
            String widgetName,
            Consumer<Double> methodToRun,
            GBSubsystem... subsystems
    ) {
        super(
                () -> SmartDashboard.putNumber(widgetName, startingValue),
                () -> methodToRun.accept(SmartDashboard.getNumber(widgetName, startingValue)),
                subsystems
        );
    }

    public DashboardCommand(
            String widgetName,
            Consumer<Double> methodToRun,
            GBSubsystem... subsystems
    ) {
        super(
                () -> SmartDashboard.putNumber(widgetName, 0),
                () -> methodToRun.accept(SmartDashboard.getNumber(widgetName, 0)),
                subsystems
        );
    }

}
