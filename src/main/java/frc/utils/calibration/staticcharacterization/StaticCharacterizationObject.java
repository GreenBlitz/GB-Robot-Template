package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

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

    private double kgPlusKs;
    private double kgMinusKs;

    public StaticCharacterizationObject(GBSubsystem subsystem, Consumer<Double> voltageConsumer,
            DoubleSupplier velocitySupplier) {
        this.subsystem = subsystem;
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
        this.kgPlusKs = 0;
        this.kgMinusKs = 0;
    }

    private void setKgPlusKs(double kgPlusKs) {
        this.kgPlusKs = kgPlusKs;
    }

    private void setKgMinusKs(double kgMinusKs) {
        this.kgMinusKs = kgMinusKs;
    }

    private double calculateKg() {
        return kgMinusKs + kgPlusKs;
    }

    private double calculateKs() {
        return kgPlusKs - calculateKg();
    }

    /**
     * Command that automatic calibrates KS. The value will be in the log in Calibration/static/.
     * IMPORTANT: the provided KS is mixed with KG. You need to do (KS - KG) to get real KS. If you're dealing with arm look at
     * "getFindKgCommand" docs. In case you don't need KG (for most rollers and wheels) you can just use the provided KS.
     *
     * @return the command
     */
    public Command getFindKsCommand() {
        return getFindKsCommand(0);
    }

    /**
     * @param startingVoltage - voltage to start from
     */
    public Command getFindKsCommand(double startingVoltage) {
        return new FindKs(subsystem, startingVoltage, voltageConsumer, velocitySupplier, this::setKgPlusKs);
    }

    /**
     * Command that automatic calibrates KG. The value will be in the log in Calibration/static/.
     * IMPORTANT: the subsystem must start still, and have the ability to go down by gravity.
     * IMPORTANT: the command return KG of the current position of the system. If you're dealing with an arm system you need
     * to multiply the provided KG by cos(currentAngleOfSystem) when 0 is in parallel to the ground to get real KG.
     *
     * @param maxStillVoltage - max voltage to keep system still
     * @return the command
     */
    public Command getFindKgCommand(double maxStillVoltage) {
        setKgPlusKs(maxStillVoltage);
        return getFindKgCommand(() -> maxStillVoltage);
    }

    private Command getFindKgCommand(DoubleSupplier stillVoltageSupplier) {
        return new SequentialCommandGroup(
                new FindKg(subsystem, stillVoltageSupplier, voltageConsumer, velocitySupplier, this::setKgMinusKs),
                new InstantCommand(() -> Logger.recordOutput(
                        StaticCharacterizationConstants.LOG_PATH + "KG OF " + subsystem.getName(), calculateKg()
                ))
        );
    }

    /**
     * Command that automatic calibrate KG and KS.
     * Please read "getFindKgCommand" and "getFindKsCommand" docs.
     *
     * @return the command
     */
    public Command getFindKsKgCommand() {
        return getFindKsKgCommand(0);
    }

    /**
     * @param startingStillVoltage - voltage to start from
     */
    public Command getFindKsKgCommand(double startingStillVoltage) {
        return new SequentialCommandGroup(
                getFindKsCommand(startingStillVoltage),
                new WaitCommand(StaticCharacterizationConstants.TIME_BETWEEN_COMMANDS),
                getFindKgCommand(() -> kgPlusKs),
                new InstantCommand(() -> Logger.recordOutput(
                        StaticCharacterizationConstants.LOG_PATH + "KS OF " + subsystem.getName(), calculateKs()
                ))
        );
    }

}
