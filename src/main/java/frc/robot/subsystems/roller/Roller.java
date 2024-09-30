package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final RollerComponents rollerComponents;
	private final RollerCommandsBuilder commandBuilder;

	public Roller(RollerComponents rollerComponents) {
		super(rollerComponents.logPath());
		this.motor = rollerComponents.motor();
		this.digitalInput = rollerComponents.digitalInput();
		this.rollerComponents = rollerComponents;
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.commandBuilder = new RollerCommandsBuilder(this);

		updateInputs();
	}

	public RollerCommandsBuilder getCommandsBuilder() {
		return commandBuilder;
	}

	public boolean isObjectIn() {
		return digitalInputsInputs.debouncedValue;
	}

	public Rotation2d getPosition() {
		return rollerComponents.positionSignal().getLatestValue();
	}

	public void updateInputs() {
		digitalInput.updateInputs(digitalInputsInputs);
		motor.updateSignals(rollerComponents.voltageSignal(), rollerComponents.positionSignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(rollerComponents.digitalInputLogPath(), digitalInputsInputs);
		Logger.recordOutput(rollerComponents.logPath() + "IsObjectIn", isObjectIn());
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

}
