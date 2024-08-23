package frc.robot.subsystems.swerve.modules.steer.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.SteerThreadInputsAutoLogged;
import frc.utils.battery.BatteryUtils;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class SimulationSteer implements ISteer {

	private final SimpleMotorSimulation motor;
	private final SimulationSteerConstants constants;


	public SimulationSteer(SimulationSteerConstants constants) {
		this.motor = constants.getMotor();
		this.constants = constants;
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return constants.getSysIdConfigInfo();
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
		motor.setControl(velocityControl.controlRequest());
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
		motorInputs.voltage = motor.getVoltage();
	}

	@Override
	public void updateInputs(SteerThreadInputsAutoLogged inputs) {
		inputs.angleOdometrySamples = new Rotation2d[] {motor.getPosition()};
	}

}
