package frc.robot.subsystems.motorsubsystem;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class MotorSubsystem extends GBSubsystem {

	ControllableMotor motor;
	GBSubsystem logPath;
	MotorCommandsBuilder motorCommandsBuilder;
	protected final SysIdCalibrator sysIdCalibrator;

	public MotorSubsystem(GBSubsystem logPath, ControllableMotor motor) {
		super(String.valueOf(logPath));
		this.motor = motor;
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, this::setPower);
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public MotorCommandsBuilder getCommandsBuilder() {
		return motorCommandsBuilder;
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public SysIdCalibrator getSysIdCalibrator() {
		return sysIdCalibrator;
	}

	public String getLogPath() {
		return String.valueOf(logPath);
	}

	public void stop() {
		motor.stop();
	}

	public void updateInputs() {
		motor.updateInputs();
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
