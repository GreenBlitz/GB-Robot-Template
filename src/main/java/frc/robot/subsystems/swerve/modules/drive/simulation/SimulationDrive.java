package frc.robot.subsystems.swerve.modules.drive.simulation;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.drive.IDrive;

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
	public void updateInputs(ModuleInputsContainer inputs) {
		inputs.getDriveMotorInputs().sysIdConfig = constants.sysIdConfig();
		inputs.getDriveMotorInputs().isConnected = true;
		inputs.getDriveMotorInputs().angle = motor.getPosition();
		inputs.getDriveMotorInputs().velocity = motor.getVelocity();
		inputs.getDriveMotorInputs().current = motor.getCurrent();
		inputs.getDriveMotorInputs().voltage = motor.getVoltage();
		inputs.getDriveMotorInputs().angleOdometrySamples = new Rotation2d[] {inputs.getDriveMotorInputs().angle};
	}

}
