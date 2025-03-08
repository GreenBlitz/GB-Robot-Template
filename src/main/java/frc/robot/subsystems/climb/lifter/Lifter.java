package frc.robot.subsystems.climb.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Lifter extends GBSubsystem {

	private final ControllableMotor motor;
	private final LifterCommandsBuilder lifterCommandsBuilder;
	private final InputSignal<Rotation2d> positionSignal;

	public Lifter(String logPath, ControllableMotor motor, InputSignal<Rotation2d> positionSignal) {
		super(logPath);

		this.motor = motor;
		this.lifterCommandsBuilder = new LifterCommandsBuilder(this);
		this.positionSignal = positionSignal;

		motor.resetPosition(LifterConstants.MINIMUM_ACHIEVABLE_POSITION);
		updateInputs();
		setDefaultCommand(lifterCommandsBuilder.stop());
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public boolean isHigher(Rotation2d position) {
		return positionSignal.isGreater(position);
	}

	public boolean isLower(Rotation2d position) {
		return !isHigher(position);
	}

	public LifterCommandsBuilder getCommandsBuilder() {
		return lifterCommandsBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		if (LifterConstants.MINIMUM_ACHIEVABLE_POSITION.getRotations() > positionSignal.getLatestValue().getRotations()) {
			motor.resetPosition(
				Rotation2d.fromDegrees(LifterConstants.MINIMUM_ACHIEVABLE_POSITION.getDegrees() + LifterConstants.RESET_OFFSET.getDegrees())
			);
		}

		motor.updateSimulation();
		updateInputs();
	}

	private void updateInputs() {
		motor.updateInputs(positionSignal);
	}

	public void applyCalibrationBindings(SmartJoystick joystick) {
		LifterStateHandler stateHandler = new LifterStateHandler(this);

		joystick.Y.onTrue(stateHandler.setState(LifterState.BACKWARD));
		joystick.X.onTrue(stateHandler.setState(LifterState.FORWARD));
		joystick.B.onTrue(stateHandler.setState(LifterState.CLIMB));
		joystick.A.onTrue(stateHandler.setState(LifterState.DEPLOY));
		joystick.R1.onTrue(stateHandler.setState(LifterState.HOLD));
	}

}
