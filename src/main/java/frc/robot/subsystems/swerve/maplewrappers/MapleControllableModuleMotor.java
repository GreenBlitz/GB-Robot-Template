package frc.robot.subsystems.swerve.maplewrappers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.swerve.maplewrappers.requests.MaplePositionRequest;
import frc.robot.subsystems.swerve.maplewrappers.requests.MapleVoltageRequest;
import frc.utils.battery.BatteryUtil;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;

public class MapleControllableModuleMotor implements ControllableMotor {

	private final SimulatedMotorController.GenericMotorController motor;
	private final SwerveModuleSimulation moduleSimulation;
	private final String logPath;

	private final PIDController positionPIDRotations;
	private boolean isPositionRequest = false;

	public MapleControllableModuleMotor(String logPath, SwerveModuleSimulation moduleSimulation, PIDController positionPIDRotations) {
		this.moduleSimulation = moduleSimulation;
		this.motor = moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60));
		;
		this.positionPIDRotations = positionPIDRotations;
		this.logPath = logPath;
	}

	public SwerveModuleSimulation getModuleSimulation() {
		return moduleSimulation;
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), false);
	}

	@Override
	public void resetPosition(Rotation2d position) {
		motor.updateControlSignal(
			Units.Degrees.of(position.getDegrees()),
			Units.RadiansPerSecond.of(0),
			Units.Degrees.of(0),
			Units.RadiansPerSecond.of(0)
		);
	}

	@Override
	public void applyRequest(IRequest<?> request) {
		isPositionRequest = false;
		positionPIDRotations.reset();

		if (request instanceof MapleVoltageRequest voltageRequest) {
			motor.requestVoltage(Units.Volt.of(voltageRequest.getSetPoint()));
		} else if (request instanceof MaplePositionRequest positionRequest) {
			isPositionRequest = true;
			positionPIDRotations.setSetpoint(positionRequest.getSetPoint().getRotations());
		}
	}

	@Override
	public void updateSimulation() {}

	@Override
	public void setBrake(boolean brake) {}

	@Override
	public void stop() {
		setPower(0);
	}

	@Override
	public void setPower(double power) {
		motor.requestVoltage(Units.Volt.of(power * BatteryUtil.DEFAULT_VOLTAGE));
	}

	@Override
	public boolean isConnected() {
		return true;
	}

	@Override
	public void updateInputs(InputSignal<?>... inputSignals) {
		if (isPositionRequest) {
			double volts = positionPIDRotations.calculate(moduleSimulation.getSteerAbsoluteFacing().getRotations());
			motor.requestVoltage(Units.Volt.of(volts));
		}

		for (InputSignal<?> inputSignal : inputSignals) {
			Logger.processInputs(logPath, inputSignal);
		}
	}

}
