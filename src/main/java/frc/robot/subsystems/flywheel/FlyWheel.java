package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

public class FlyWheel extends GBSubsystem {

	private final ControllableMotor masterMotor;

	private final IRequest<Rotation2d> velocityRequest;
	private final IRequest<Double> voltageRequest;

	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Rotation2d> velocitySignal;

	private final FlyWheelCommandBuilder flyWheelCommandBuilder;

	public FlyWheel(
		String logPath,
		IRequest<Rotation2d> velocityRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Double> voltageSignal,
		ControllableMotor masterMotor
	) {
		super(logPath);
		this.velocityRequest = velocityRequest;
		this.voltageRequest = voltageRequest;
		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
		this.masterMotor = masterMotor;
		this.flyWheelCommandBuilder = new FlyWheelCommandBuilder(this);
	}

	public FlyWheelCommandBuilder getCommandBuilder() {
		return flyWheelCommandBuilder;
	}

	public void setVoltage(double voltage) {
		masterMotor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public void setVelocity(Rotation2d velocity) {
		masterMotor.applyRequest(velocityRequest.withSetPoint(velocity));
	}

	public Rotation2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public boolean isAtVelocity(Rotation2d chekVelocity) {
		return ToleranceMath.isNear(getVelocity().getRotations(), chekVelocity.getRotations(), Constants.VELOCITY_TOLERANCE);
	}

	public void stop() {
		masterMotor.stop();
	}

	public void setBrake(boolean brake) {
		masterMotor.setBrake(brake);
	}

	@Override
	protected void subsystemPeriodic() {
		masterMotor.updateInputs(velocitySignal, voltageSignal);
		masterMotor.updateSimulation();
		Logger.recordOutput(getLogPath() + "/targetVelocity", velocityRequest.getSetPoint());
	}

}
