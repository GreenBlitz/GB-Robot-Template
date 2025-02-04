package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.factory.KrakenX60ArmBuilder;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;


public class Arm extends GBSubsystem {

	private final ControllableMotor motor;
	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;
	private final InputSignal<Rotation2d> motorPositionSignal;
	private final InputSignal<Double> motorVoltageSignal;
	private final IAngleEncoder encoder;
	private final InputSignal<Rotation2d> encoderPositionSignal;
	private final ArmCommandsBuilder commandsBuilder;
	private final SysIdCalibrator sysIdCalibrator;

	public Arm(
		String logPath,
		ControllableMotor motor,
		IRequest<Rotation2d> positionRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> motorPositionSignal,
		InputSignal<Double> motorVoltageSignal,
		IAngleEncoder encoder,
		InputSignal<Rotation2d> encoderPositionSignal
	) {
		super(logPath);
		this.motor = motor;
		this.positionRequest = positionRequest;
		this.voltageRequest = voltageRequest;
		this.motorPositionSignal = motorPositionSignal;
		this.motorVoltageSignal = motorVoltageSignal;
		this.encoder = encoder;
		this.encoderPositionSignal = encoderPositionSignal;
		this.commandsBuilder = new ArmCommandsBuilder(this);
		this.sysIdCalibrator = new SysIdCalibrator(
			motor.getSysidConfigInfo(),
			this,
			voltage -> setVoltage(voltage + KrakenX60ArmBuilder.kG * getPosition().getCos())
		);

		periodic();
		setDefaultCommand(getCommandsBuilder().stayInPlace());
	}

	public ArmCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public Rotation2d getPosition() {
		return motorPositionSignal.getLatestValue();
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateSimulation();
		updateInputs();
	}

	private void updateInputs() {
		motor.updateInputs(motorPositionSignal, motorVoltageSignal);
		encoder.updateInputs(encoderPositionSignal);
	}

	protected void resetByEncoderPosition() {
		motor.resetPosition(encoderPositionSignal.getLatestValue());
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void stop() {
		motor.stop();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void setTargetPosition(Rotation2d position) {
		motor.applyRequest(positionRequest.withSetPoint(position));
	}

	protected void stayInPlace() {
		setTargetPosition(motorPositionSignal.getLatestValue());
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return motorPositionSignal.isNear(position, tolerance);
	}

	public void applyCalibrationBindings(SmartJoystick joystick) {
		// calibrate kG using phoenix tuner by setting the voltage

		// check limits
		joystick.A.onTrue(commandsBuilder.setPower(joystick.getAxisValue(Axis.LEFT_Y)));

		// calibrate PID using phoenix tuner and these bindings:
		joystick.POV_UP.onTrue(commandsBuilder.moveToPosition(Rotation2d.fromDegrees(-40)));
		joystick.POV_DOWN.onTrue(commandsBuilder.moveToPosition(Rotation2d.fromDegrees(0)));
		joystick.POV_LEFT.onTrue(commandsBuilder.moveToPosition(Rotation2d.fromDegrees(90)));
		joystick.POV_RIGHT.onTrue(commandsBuilder.moveToPosition(Rotation2d.fromDegrees(200)));

		// calibrate feed forward using sys id:
		sysIdCalibrator.setAllButtonsForCalibration(joystick);

		// calibrate max acceleration and cruse velocity by the equations: max acceleration = (12 + Ks)/2kA, cruse velocity = (12 + Ks)/kV
	}

}
