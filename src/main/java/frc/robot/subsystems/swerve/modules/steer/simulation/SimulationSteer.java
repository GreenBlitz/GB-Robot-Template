package frc.robot.subsystems.swerve.modules.steer.simulation;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.SteerInputsAutoLogged;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class SimulationSteer implements ISteer {

	private final SimpleMotorSimulation motor;
	private final SimulationSteerConstants constants;

	private final PositionVoltage positionRequest;
	private final VoltageOut voltageRequest;

	public SimulationSteer(SimulationSteerConstants constants) {
		this.motor = constants.getMotor();
		this.constants = constants;

		this.positionRequest = new PositionVoltage(0).withEnableFOC(constants.getEnableFOC());
		this.voltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOC());
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return constants.getSysIdConfigInfo();
	}

	@Override
	public void setBrake(boolean brake) {}

	@Override
	public void resetToAngle(Rotation2d angle) {}


	@Override
	public void stop() {
		motor.stop();
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setControl(voltageRequest.withOutput(voltage));
	}

	@Override
	public void setTargetAngle(Rotation2d angle) {
		motor.setControl(positionRequest.withPosition(angle.getRotations()));
	}


	@Override
	public void updateInputs(SteerInputsAutoLogged inputs) {
		inputs.isConnected = true;
		inputs.angle = motor.getPosition();
		inputs.velocity = motor.getVelocity();
		inputs.voltage = motor.getVoltage();
		inputs.angleOdometrySamples = new Rotation2d[] {inputs.angle};
	}

}
