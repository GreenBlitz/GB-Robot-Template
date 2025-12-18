package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.*;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.battery.BatteryUtil;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

public class Arm extends GBSubsystem {

	protected final ControllableMotor motor;
	private final ArmSignals signals;
	private final IRequest<Double> voltageRequest;
	private final IFeedForwardRequest positionRequest;
	private final SysIdCalibrator sysIdCalibrator;
	private final double kG;
	private final ArmCommandBuilder commandBuilder;

	public Arm(
		String logPath,
		ControllableMotor motor,
		ArmSignals signals,
		IRequest<Double> voltageRequest,
		IFeedForwardRequest positionRequest,
		double kG
	) {
		super(logPath);
		this.motor = motor;
		this.signals = signals;
		this.voltageRequest = voltageRequest;
		this.positionRequest = positionRequest;
		this.kG = kG;
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, (voltage) -> setVoltage(voltage + getKgVoltage()));
		commandBuilder = new ArmCommandBuilder(this);
		setDefaultCommand(commandBuilder.stayInPlace());
	}

	public ArmCommandBuilder getCommandsBuilder() {
		return commandBuilder;
	}

	public double getVoltage() {
		return signals.voltage().getLatestValue();
	}

	public double getCurrent() {
		return signals.current().getLatestValue();
	}

	public Rotation2d getVelocity() {
		return signals.velocity().getLatestValue();
	}

	public Rotation2d getPosition() {
		return signals.position().getLatestValue();
	}

	public SysIdCalibrator getSysIdCalibrator() {
		return sysIdCalibrator;
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d tolerance) {
		return signals.position().isNear(targetPosition, tolerance);
	}

	public boolean isPastPosition(Rotation2d position) {
		return signals.position().isGreater(position);
	}

	public boolean isBehindPosition(Rotation2d position) {
		return signals.position().isLess(position);
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateSimulation();
		updateInputs();
		log();
	}

	private void updateInputs() {
		motor.updateInputs(signals.voltage(), signals.current(), signals.velocity(), signals.position());
	}

	public void log() {
		Logger.recordOutput(getLogPath() + "/PositionTarget", positionRequest.getSetPoint());
		Logger.recordOutput(getLogPath() + "/ArbitraryFeedForward", positionRequest.getArbitraryFeedForward());
	}

	public void setVoltage(Double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public void setTargetPosition(Rotation2d targetPosition) {
		motor.applyRequest(positionRequest.withSetPoint(targetPosition));
	}

	public void setPosition(Rotation2d targetPosition) {
		motor.resetPosition(targetPosition);
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void setFeedForward(double arbitraryFeedForward) {
		positionRequest.withArbitraryFeedForward(arbitraryFeedForward);
	}

	public void stayInPlace() {
		setTargetPosition(signals.position().getLatestValue());
	}

	public double getKgVoltage() {
		return Robot.ROBOT_TYPE.isReal() ? kG * getPosition().getCos() : 0;
	}

	public void applyCalibrationBindings(SmartJoystick joystick, double maxCalibrationPower) {
		joystick.POV_DOWN.onTrue(new InstantCommand(() -> commandBuilder.setIsSubsystemRunningIndependently(true)));
		joystick.POV_UP.onTrue(new InstantCommand(() -> commandBuilder.setIsSubsystemRunningIndependently(false)));

		// Calibrate kG using phoenix tuner by setting the voltage

		// Check limits
		joystick.R1.whileTrue(
			commandBuilder
				.setPower(() -> joystick.getAxisValue(Axis.LEFT_Y) * maxCalibrationPower + (getKgVoltage() / BatteryUtil.getCurrentVoltage()))
		);

		// Calibrate feed forward using sys id:
		sysIdCalibrator.setAllButtonsForCalibration(joystick);
	}

}

