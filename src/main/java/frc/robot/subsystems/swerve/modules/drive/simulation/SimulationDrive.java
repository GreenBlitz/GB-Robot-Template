package frc.robot.subsystems.swerve.modules.drive.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.drive.DriveThreadMetersInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.utils.battery.BatteryUtils;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class SimulationDrive implements IDrive {

	private final SimpleMotorSimulation motor;
	private final SimulationDriveConstants constants;

	public SimulationDrive(SimulationDriveConstants constants) {
		this.motor = constants.motorSimulation();
		this.constants = constants;
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return constants.sysIdConfigInfo();
	}

	@Override
	public void setBrake(boolean brake) {}

	@Override
	public void resetAngle(Rotation2d angle) {}

	@Override
	public void stop() {
		motor.stop();
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setPower(voltage / BatteryUtils.DEFAULT_VOLTAGE);
	}


	@Override
	public void setTargetVelocity(CloseLoopControl velocityControl) {
		double voltage = ModuleUtils.velocityToVoltage(
			velocityControl.targetSetPoint(),
			constants.maxVelocityPerSecond(),
			ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
		);
		setVoltage(voltage);
	}

	@Override
	public void setTargetAngle(CloseLoopControl positionControl) {
		motor.setControl(positionControl.controlRequest());
	}

	@Override
	public void updateInputs(MotorInputsAutoLogged motorInputs) {
		motorInputs.connected = true;
		motorInputs.angle = motor.getPosition();
		motorInputs.velocity = motor.getVelocity();
		motorInputs.current = motor.getCurrent();
		motorInputs.voltage = motor.getVoltage();
	}

	@Override
	public void updateInputs(DriveThreadMetersInputsAutoLogged inputs) {
		inputs.angleOdometrySamples = new Rotation2d[] {motor.getPosition()};
	}

}
