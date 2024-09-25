package frc.robot.subsystems.intake;

import frc.robot.hardware.digitalinput.DigitalInputInputs;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;

public class Intake extends GBSubsystem {
	
	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged inputs;
	private final InputSignal<Double> voltage;
	
	public Intake(String logPath, IMotor motor, IDigitalInput digitalInput, InputSignal<Double> voltage) {
		super(logPath);
		this.motor = motor;
		this.digitalInput = digitalInput;
		this.voltage = voltage;
		inputs = new DigitalInputInputsAutoLogged();
	}
	
	public void setPower(double power){
		motor.setPower(power);
	}
	
	public
	
	public void updateInputs(){
		digitalInput.updateInputs(inputs);
	}
	
	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}
}
