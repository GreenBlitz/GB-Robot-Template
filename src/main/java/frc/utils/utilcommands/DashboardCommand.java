package frc.utils.utilcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.GBSubsystem;

import java.util.function.Consumer;

/**
 * This Command run a consumer through dashboard value
 */
public class DashboardCommand extends InitExecuteCommand {

    private static final double DEFAULT_VALUE = 0;

    public DashboardCommand(double startingValue, String widgetName, Consumer<Double> methodToRun, GBSubsystem... subsystems) {
        super(
                () -> SmartDashboard.putNumber(widgetName, startingValue),
                () -> methodToRun.accept(SmartDashboard.getNumber(widgetName, startingValue)),
                subsystems
        );
    }

    public DashboardCommand(String widgetName, Consumer<Double> methodToRun, GBSubsystem... subsystems) {
        super(
                () -> SmartDashboard.putNumber(widgetName, DEFAULT_VALUE),
                () -> methodToRun.accept(SmartDashboard.getNumber(widgetName, DEFAULT_VALUE)),
                subsystems
        );
    }

}
