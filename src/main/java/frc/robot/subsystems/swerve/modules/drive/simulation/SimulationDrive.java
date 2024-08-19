package frc.robot.subsystems.swerve.modules.drive.simulation;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.drive.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class SimulationDrive implements IDrive {

	private final SimpleMotorSimulation motor;
	private final SimulationDriveConstants constants;
	private final VoltageOut voltageRequest;

	public SimulationDrive(SimulationDriveConstants constants) {
		this.motor = constants.motorSimulation();
		this.constants = constants;
		this.voltageRequest = new VoltageOut(0).withEnableFOC(constants.enableFOC());
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return constants.sysIdConfigInfo();
	}

	@Override
	public void setBrake(boolean brake) {}


	@Override
	public void stop() {
		motor.stop();
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setControl(voltageRequest.withOutput(voltage));
	}

	@Override
	public void setTargetVelocity(Rotation2d velocityPerSecond) {
		double voltage = ModuleUtils
			.velocityToVoltage(velocityPerSecond, constants.maxVelocityPerSecond(), ModuleConstants.VOLTAGE_COMPENSATION_SATURATION);
		setVoltage(voltage);
	}


	@Override
	public void updateInputs(DriveInputsAutoLogged inputs) {
		inputs.isConnected = true;
		inputs.angle = motor.getPosition();
		inputs.velocity = motor.getVelocity();
		inputs.current = motor.getCurrent();
		inputs.voltage = motor.getVoltage();
		inputs.angleOdometrySamples = new Rotation2d[] {inputs.angle};
	}

}
