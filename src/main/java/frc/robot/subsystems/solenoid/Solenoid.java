package frc.robot.subsystems.solenoid;

import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;

public class Solenoid extends GBSubsystem {
	
	private final IMotor motor;
	private final InputSignal<Double> voltageSignal;
	
	public Solenoid(SolenoidComponents solenoidComponents) {
		super(solenoidComponents.logPath());
		this.motor = solenoidComponents.solenoid();;
		this.voltageSignal = solenoidComponents.voltageSignal();
	}
	
	@Override
	protected void subsystemPeriodic() {
		motor.updateSignals(voltageSignal);
	}
	
	protected void stop() {
		motor.stop();
	}
	
	protected void setPower(double power) {
		motor.setPower(power);
	}
	
}
