package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

public class FlyWheel extends GBSubsystem {

	private final ControllableMotor motor;

	private final IRequest<Rotation2d> velocityRequest;
	private final IRequest<Double> voltageRequest;

	private final InputSignal<Rotation2d> velocitySignal;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;

	private final FlyWheelCommandBuilder flyWheelCommandBuilder;

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
		joystick.A.onTrue(getCommandBuilder().setTargetVelocity(Rotation2d.fromRotations(1)));
		joystick.A.onTrue(getCommandBuilder().setTargetVelocity(Rotation2d.fromRotations(2)));
	}

}
