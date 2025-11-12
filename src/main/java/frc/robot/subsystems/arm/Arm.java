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
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Rotation2d> velocitySignal;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal<Double> currentSignal;
	private final IRequest<Double> armVoltageRequest;
	private final IFeedForwardRequest armPositionRequest;
	private final SysIdCalibrator sysIdCalibrator;
	private final double kG;
	private final ArmCommandBuilder armCommandBuilder;


	public Arm(
		String logPath,
		ControllableMotor motor,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Rotation2d> positionSignal,
		IRequest<Double> armVoltageRequest,
		IFeedForwardRequest armPositionRequest,
		SysIdCalibrator.SysIdConfigInfo config,
		double kG
	) {
		super(logPath);
		this.motor = motor;
		this.positionSignal = positionSignal;
		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.armVoltageRequest = armVoltageRequest;
		this.armPositionRequest = armPositionRequest;
		this.kG = kG;
		sysIdCalibrator = new SysIdCalibrator(config, this, this::setVoltage);
		armCommandBuilder = new ArmCommandBuilder(this);
		setDefaultCommand(armCommandBuilder.stayInPlace());
	}

	public ArmCommandBuilder getCommandsBuilder() {
		return armCommandBuilder;
	}

	public Rotation2d getPosition() {
		return positionSignal.getLatestValue();
	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public Rotation2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public double getCurrent() {
		return currentSignal.getLatestValue();
	}

	public SysIdCalibrator getSysIdCalibrator() {
		return sysIdCalibrator;
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d tolerance) {
		return positionSignal.isNear(targetPosition, tolerance);
	}

	public boolean isPastPosition(Rotation2d position) {
		return positionSignal.isGreater(position);
	}

	public boolean isBehindPosition(Rotation2d position) {
		return positionSignal.isLess(position);
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateSimulation();
		updateInputs();
		log();
	}

	private void updateInputs() {
		motor.updateInputs(voltageSignal, currentSignal, velocitySignal, positionSignal);
	}

	public void log() {
		Logger.recordOutput(getLogPath() + "PositionTarget/", armPositionRequest.getSetPoint());
	}

	public void setVoltage(Double voltage) {
		motor.applyRequest(armVoltageRequest.withSetPoint(voltage));
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public void setTargetPosition(Rotation2d targetPosition) {
		motor.applyRequest(armPositionRequest.withSetPoint(targetPosition));
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void setFeedForward(double arbitraryFeedForward) {
		armPositionRequest.withArbitraryFeedForward(arbitraryFeedForward);
	}

	protected void stayInPlace() {
		setTargetPosition(positionSignal.getLatestValue());
	}

	private double getKgVoltage() {
		return Robot.ROBOT_TYPE.isReal() ? kG * getPosition().getCos() : 0;
	}

	public void applyCalibrationBindings(SmartJoystick joystick, double maxCalibrationPower) {
		joystick.A.onTrue(new InstantCommand(() -> armCommandBuilder.setIsSubsystemRunningIndependently(true)));
		joystick.B.onTrue(new InstantCommand(() -> armCommandBuilder.setIsSubsystemRunningIndependently(false)));

		// Calibrate kG using phoenix tuner by setting the voltage

		// Check limits
		joystick.R1.whileTrue(
			armCommandBuilder
				.setPower(() -> joystick.getAxisValue(Axis.LEFT_Y) * maxCalibrationPower + (getKgVoltage() / BatteryUtil.getCurrentVoltage()))
		);

		// Calibrate feed forward using sys id:
		sysIdCalibrator.setAllButtonsForCalibration(joystick);
	}

}

