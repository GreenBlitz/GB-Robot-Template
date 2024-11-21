package frc.robot.subsystems;

import frc.robot.hardware.interfaces.ControllableMotor;

public class MotorSubsystemMaya extends GBSubsystem {

	public ControllableMotor motor;
	public final MotorCommandBuilder motorCommandBuilder;

	public MotorSubsystemMaya(ControllableMotor motor, String logPath, MotorCommandBuilder motorCommandBuilder) {
		super(logPath);
		this.motor = motor;
		this.motorCommandBuilder = motorCommandBuilder;
	}

	public MotorCommandBuilder getCommandsBuilder() {
		return motorCommandBuilder;
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateInputs();
	}

}
