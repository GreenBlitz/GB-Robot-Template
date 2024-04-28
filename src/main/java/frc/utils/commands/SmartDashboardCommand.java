package frc.utils.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.GBSubsystem;

import java.util.function.Consumer;

/**
 * This Command run a consumer through dashboard value
 */
public class SmartDashboardCommand extends InitExecuteCommand {

    public SmartDashboardCommand(
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

}
