package frc.robot.subsystems.solenoid;

import frc.robot.hardware.interfaces.IMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Solenoid extends GBSubsystem {

	private final IMotor motor;
	private final InputSignal<Double> voltageSignal;
	private final SolenoidCommandBuilder commandsBuilder;

	public Solenoid(String logPath, IMotor motor, InputSignal<Double> voltageSignal) {
		super(logPath);
		this.motor = motor;
		this.voltageSignal = voltageSignal;
		this.commandsBuilder = new SolenoidCommandBuilder(this);

		updateInputs();
	}

	public SolenoidCommandBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void updateInputs() {
		motor.updateInputs(voltageSignal);
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
