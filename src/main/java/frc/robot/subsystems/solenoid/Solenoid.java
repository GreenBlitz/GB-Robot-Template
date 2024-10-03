package frc.robot.subsystems.solenoid;

import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;

public class Solenoid extends GBSubsystem {

	private final IMotor motor;
	private final InputSignal<Double> voltageSignal;
	private final SolenoidCommandsBuilder commandsBuilder;

	public Solenoid(SolenoidComponents solenoidComponents) {
		super(solenoidComponents.logPath());
		this.motor = solenoidComponents.solenoid();
		this.voltageSignal = solenoidComponents.voltageSignal();
		this.commandsBuilder = new SolenoidCommandsBuilder(this);

		updateInputs();
	}

	public SolenoidCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void updateInputs() {
		motor.updateSignals(voltageSignal);
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	protected void stop() {
		motor.stop();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

}
