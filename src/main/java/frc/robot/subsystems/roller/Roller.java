package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;


public class Roller extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final RollerStuff rollerStuff;
	private final RollerCommandsBuilder commandBuilder;
	private Rotation2d targetPosition;

	public Roller(RollerStuff rollerStuff) {
		super(rollerStuff.logPath());
		this.motor = rollerStuff.motor();
		this.digitalInput = rollerStuff.digitalInput();
		this.rollerStuff = rollerStuff;
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();

		this.commandBuilder = new RollerCommandsBuilder(this);
		this.targetPosition = new Rotation2d();
		updateInputs();
	}

	public RollerCommandsBuilder getCommandsBuilder() {
		return commandBuilder;
	}

	public boolean isObjectIn() {
		return digitalInputsInputs.debouncedValue;
	}

	public void updateInputs() {
		digitalInput.updateInputs(digitalInputsInputs);
		motor.updateSignals(rollerStuff.voltageSignal(), rollerStuff.positionSignal());
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

	public Rotation2d getPosition() {
		return rollerStuff.positionSignal().getLatestValue();
	}

	protected void setTargetPosition(double rotations) {
		this.targetPosition = Rotation2d.fromRotations(getPosition().getRotations() + rotations);
	}

	protected boolean isPastPosition() {
		return getPosition().getRotations() > targetPosition.getRotations();
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(rollerStuff.digitalInputLogPath(), digitalInputsInputs);
		Logger.recordOutput(rollerStuff.logPath() + "IsObjectIn", isObjectIn());
	}

}
