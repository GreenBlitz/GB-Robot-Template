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

public class Arm extends GBSubsystem {

	private final ControllableMotor arm;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Rotation2d> velocitySignal;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal<Double> currentSignal;
	private final ArmCommandBuilder armCommandBuilder;
	private final IRequest<Double> armVoltageRequest;
	private final IFeedForwardRequest motionMagicRequest;
	private final SysIdCalibrator sysIdCalibrator;
	private final Double kG;


	public Arm(
		String logPath,
		ControllableMotor arm,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		IRequest<Double> armVoltageRequest,
		IFeedForwardRequest motionMagicRequest,
		SysIdCalibrator.SysIdConfigInfo config,
        Double kG
	) {
		super(logPath);
		this.arm = arm;
		this.positionSignal = positionSignal;
		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.armVoltageRequest = armVoltageRequest;
		this.motionMagicRequest = motionMagicRequest;
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
		arm.updateSimulation();
		updateInputs();
	}

	private void updateInputs() {
		arm.updateInputs(positionSignal, voltageSignal, velocitySignal, currentSignal);
	}

	public void log() {}

	public void setVoltage(Double voltage) {
		arm.applyRequest(armVoltageRequest.withSetPoint(voltage));
	}

	public void setBrake(boolean brake) {
		arm.setBrake(brake);
	}

	public void setTargetPosition(Rotation2d targetPosition) {
		arm.applyRequest(motionMagicRequest.withSetPoint(targetPosition));
	}

	public void setPower(double power) {
		arm.setPower(power);
	}

	protected void stayInPlace() {
		setTargetPosition(positionSignal.getLatestValue());
	}

	private double getKgVoltage() {
		return Robot.ROBOT_TYPE.isReal() ? kG * getPosition().getCos() : 0;
	}

	public void applyCalibrationBindings(SmartJoystick joystick) {
		joystick.A.onTrue(new InstantCommand(() -> armCommandBuilder.setIsSubsystemRunningIndependently(true)));
		joystick.B.onTrue(new InstantCommand(() -> armCommandBuilder.setIsSubsystemRunningIndependently(false)));

		// Calibrate kG using phoenix tuner by setting the voltage

		// Check limits
		joystick.R1.whileTrue(
			armCommandBuilder.setPower(
				() -> joystick.getAxisValue(Axis.LEFT_Y) * ArmConstants.CALIBRATION_MAX_POWER
					+ (getKgVoltage() / BatteryUtil.getCurrentVoltage())
			)
		);

		// Calibrate feed forward using sys id:
		sysIdCalibrator.setAllButtonsForCalibration(joystick);

//        ArmStateHandler armStateHandler = new ArmStateHandler(this, () -> 0.0, () -> 0.0);
//
//        // Calibrate PID using phoenix tuner and these bindings:
//        joystick.POV_UP.onTrue(armStateHandler.setState(ArmState.CLOSED));
//        joystick.POV_RIGHT.onTrue(armStateHandler.setState(ArmState.CLIMB));
//        joystick.POV_LEFT.onTrue(armStateHandler.setState(ArmState.NET));
//        joystick.POV_DOWN.onTrue(armStateHandler.setState(ArmState.PRE_L4));

		// Calibrate max acceleration and cruise velocity by the equations: max acceleration = (12 + Ks)/2kA, cruise velocity =(12 + Ks)/kV
	}

}

