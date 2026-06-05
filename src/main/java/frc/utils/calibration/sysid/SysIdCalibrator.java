package frc.utils.calibration.sysid;

import com.ctre.phoenix6.SignalLogger;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.robot.subsystems.GBSubsystem;
import frc.joysticks.SmartJoystick;

import java.util.function.Consumer;

import static org.wpilib.units.Units.Volts;

public class SysIdCalibrator {

	private final SysIdRoutine sysIdRoutine;
	private final GBSubsystem usedSubsystem;
	private final boolean isCTRE;

	public record SysIdConfigInfo(SysIdRoutine.Config config, boolean isCTRE) {}

	/**
	 * IMPORTANT: You must do SignalLogger.stop() at the end of the calibration
	 *
	 * @param voltageSetControl - note that this function needs to use kg in it so the mechanism won't move because of gravity.
	 */
	public SysIdCalibrator(SysIdConfigInfo sysIdConfigInfo, GBSubsystem subsystem, Consumer<Double> voltageSetControl) {
		this.usedSubsystem = subsystem;
		this.isCTRE = sysIdConfigInfo.isCTRE;

		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
			(Voltage voltage) -> voltageSetControl.accept(voltage.in(Volts)),
			null,
			usedSubsystem,
			usedSubsystem.getName()
		);

		this.sysIdRoutine = new SysIdRoutine(sysIdConfigInfo.config, mechanism);
	}

	/**
	 * Sets for you all the buttons you need to do sysid calibration. The buttons are ordered by the click order. IMPORTANT: You must do
	 * SignalLogger.stop() at the end of the calibration
	 *
	 * @param smartJoystick - the joystick to apply the buttons on
	 */
	public void setAllButtonsForCalibration(SmartJoystick smartJoystick) {
		smartJoystick.A.whileTrue(getSysIdCommand(true, SysIdRoutine.Direction.kForward));
		smartJoystick.B.whileTrue(getSysIdCommand(true, SysIdRoutine.Direction.kReverse));
		smartJoystick.X.whileTrue(getSysIdCommand(false, SysIdRoutine.Direction.kForward));
		smartJoystick.Y.whileTrue(getSysIdCommand(false, SysIdRoutine.Direction.kReverse));
		smartJoystick.START.onTrue(new InstantCommand(SignalLogger::stop));
	}

	public Command getSysIdCommand(boolean isQuasistatic, SysIdRoutine.Direction direction) {
		return isQuasistatic ? getSysIdQuasistatic(direction) : getSysIdDynamic(direction);
	}

	public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
		Command command = sysIdRoutine.quasistatic(direction);
		return getAppropriateCommand(command);
	}

	public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
		Command command = sysIdRoutine.dynamic(direction);
		return getAppropriateCommand(command);
	}

	private Command getAppropriateCommand(Command sysIdCommand) {
		sysIdCommand.addRequirements(usedSubsystem);
		return isCTRE ? getCTRECommand(sysIdCommand) : sysIdCommand;
	}

	private Command getCTRECommand(Command sysIdCommand) {
		return new SequentialCommandGroup(new InstantCommand(SignalLogger::start), sysIdCommand);
	}

}
