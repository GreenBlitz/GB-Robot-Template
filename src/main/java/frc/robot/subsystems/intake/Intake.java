package frc.robot.subsystems.intake;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends GBSubsystem {
	
	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final InputSignal<Double> voltageSignal;
	
	public Intake(IntakeStuff intakeStuff) {
		super(intakeStuff.logPath());
		
		this.motor = intakeStuff.motor();
		this.digitalInput = intakeStuff.digitalInput();
		this.voltageSignal = intakeStuff.inputSignal();
		
		digitalInputsInputs = new DigitalInputInputsAutoLogged();
	}
	
	public void setPower(double power) {
		motor.setPower(power);
	}
	
	public void stop() {
		motor.stop();
	}
	
	public void updateInputs() {
		digitalInput.updateInputs(digitalInputsInputs);
	}
	
	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(getLogPath() + "digital inputs", digitalInputsInputs);
	}
}
