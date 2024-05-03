package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.utils.GBSubsystem;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Interface to automatic calibration for KG and KS.
 * KG - minimal voltage to face gravity
 * KS - voltage to face static friction. If you add to it more voltage the system should move
 */
public class StaticCharacterizationObject {

    private final GBSubsystem subsystem;

    private final Consumer<Double> voltageConsumer;

    private final DoubleSupplier velocitySupplier;

    private double ks;

    public StaticCharacterizationObject(GBSubsystem subsystem, Consumer<Double> voltageConsumer,
            DoubleSupplier velocitySupplier) {
        this.subsystem = subsystem;
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
        this.ks = 0;
    }

    private void setKs(double ks) {
        this.ks = ks;
    }

    /**
     * Command that automatic calibrates KS. The value will be in the log in Calibration/static/.
     * IMPORTANT: the provided KS is mixed with KG. You need to do (KS - KG) to get real KS. If you're dealing with arm look at
     * "getFindKgCommand" docs. In case you don't need KG (for most rollers and wheels) you can just use the provided KS.
     *
     * @return the command
     */
    public Command getFindKsCommand() {
        return new FindKs(subsystem, voltageConsumer, velocitySupplier, this::setKs);
    }

    /**
     * Command that automatic calibrates KG. The value will be in the log in Calibration/static/.
     * IMPORTANT: the subsystem must start still, and have the ability to go down by gravity.
     * IMPORTANT: the command return KG of the current position of the system. If you're dealing with an arm system you need
     * to multiply the provided KG by cos(currentAngleOfSystem) when 0 is in parallel to the ground to get real KG.
     *
     * @param stillVoltage - voltage to keep system still
     * @return the command
     */
    public Command getFindKgCommand(double stillVoltage) {
        return getFindKgCommand(() -> stillVoltage);
    }

    private Command getFindKgCommand(DoubleSupplier stillVoltageSupplier) {
        return new FindKg(subsystem, stillVoltageSupplier, voltageConsumer, velocitySupplier);
    }

    /**
     * Command that automatic calibrate KG and KS.
     * Please read "getFindKgCommand" and "getFindKsCommand" docs.
     *
     * @return the command
     */
    public Command getFindKsKgCommand() {
        return new SequentialCommandGroup(
                getFindKsCommand(),
                new WaitCommand(StaticCharacterizationConstants.TIME_BETWEEN_COMMANDS),
                getFindKgCommand(() -> ks)
        );
    }

}
