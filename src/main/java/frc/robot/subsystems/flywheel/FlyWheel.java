package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;


public class FlyWheel extends GBSubsystem {

	private final IRequest<Rotation2d> velocityRequest;
	private final IRequest<Double> voltageRequest;

	private final InputSignal<Rotation2d> velocitySignal;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;

	private final ControllableMotor motor;

	private final FlyWheelCommandBuilder flyWheelCommandBuilder;

	private final SysIdCalibrator sysIdCalibrator;

	public FlyWheel(
		String logPath,
		IRequest<Rotation2d> velocityRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		ControllableMotor motor
	) {
		super(logPath);
		this.velocityRequest = velocityRequest;
		this.voltageRequest = voltageRequest;
		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.motor = motor;
		this.flyWheelCommandBuilder = new FlyWheelCommandBuilder(this);
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, this::setVoltage);
		setDefaultCommand(getCommandBuilder().stop());
	}

	public FlyWheelCommandBuilder getCommandBuilder() {
		return flyWheelCommandBuilder;
	}

	public void setTargetVelocity(Rotation2d velocity) {
		motor.applyRequest(velocityRequest.withSetPoint(velocity));
	}

	public void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public Rotation2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public double getCurrent() {
		return currentSignal.getLatestValue();
	}

	public boolean isAtVelocity(Rotation2d targetVelocity, double tolerance) {
		return ToleranceMath.isNear(getVelocity().getRotations(), targetVelocity.getRotations(), tolerance);
	}

	public void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateSimulation();
		motor.updateInputs(velocitySignal, voltageSignal, currentSignal);
		Logger.recordOutput(getLogPath() + "/targetVelocity", velocityRequest.getSetPoint());
	}

	public void applyCalibrationsBindings(SmartJoystick joystick) {
		joystick.L1.onTrue(new InstantCommand(() -> getCommandBuilder().setIsSubsystemRunningIndependently(true)));
		joystick.L3.onTrue(new InstantCommand(() -> getCommandBuilder().setIsSubsystemRunningIndependently(false)));
		joystick.POV_LEFT.onTrue(getCommandBuilder().setTargetVelocity(Rotation2d.fromRotations(1)));
		joystick.POV_UP.onTrue(getCommandBuilder().setTargetVelocity(Rotation2d.fromRotations(2)));
		joystick.POV_DOWN.onTrue(getCommandBuilder().stop());
		sysIdCalibrator.setAllButtonsForCalibration(joystick);
	}

}
