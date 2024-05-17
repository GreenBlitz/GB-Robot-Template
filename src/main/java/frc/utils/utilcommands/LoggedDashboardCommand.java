package frc.utils.utilcommands;

import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.Consumer;

/**
 * This Command run a consumer through dashboard value
 */
public class LoggedDashboardCommand extends InitExecuteCommand {

    private static final double DEFAULT_VALUE = 0;

    public LoggedDashboardCommand(String widgetName, Consumer<Double> methodToRun, GBSubsystem... subsystems) {
        this(DEFAULT_VALUE, widgetName, methodToRun, subsystems);
    }

    public LoggedDashboardCommand(double startingValue, String widgetName, Consumer<Double> methodToRun,
            GBSubsystem... subsystems) {
        this(new LoggedDashboardNumber(widgetName, startingValue), methodToRun, subsystems);
    }

    public LoggedDashboardCommand(LoggedDashboardNumber logDashboardNumber, Consumer<Double> methodToRun,
            GBSubsystem... subsystems) {
        super(
                () -> {},
                () -> methodToRun.accept(logDashboardNumber.get()),
                subsystems
        );
    }

}
