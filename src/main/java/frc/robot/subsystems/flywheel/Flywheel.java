package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.utils.GBSubsystem;

public class Flywheel extends GBSubsystem {

	private final FlywheelComponents topFlywheelComponents;
	private final FlywheelComponents bottomFlywheelComponents;
	private final ControllableMotor topMotor;
	private final ControllableMotor bottomMotor;
	private final FlywheelCommandsBuilder commandsBuilder;

	public Flywheel(FlywheelComponents topFlywheelComponents, FlywheelComponents bottomFlywheelComponents, String logPath) {
		super(logPath);

		this.topFlywheelComponents = topFlywheelComponents;
		this.bottomFlywheelComponents = bottomFlywheelComponents;
		this.topMotor = topFlywheelComponents.motor();
		this.bottomMotor = bottomFlywheelComponents.motor();
		this.commandsBuilder = new FlywheelCommandsBuilder(this);

		updateInputs();
	}

	public FlywheelCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	protected void setPower(double power) {
		topMotor.setPower(power);
		bottomMotor.setPower(power);
	}

	protected void setTargetVelocity(Rotation2d targetVelocity) {
		topMotor.applyAngleRequest(topFlywheelComponents.velocityRequest().withSetPoint(targetVelocity));
		bottomMotor.applyAngleRequest(bottomFlywheelComponents.velocityRequest().withSetPoint(targetVelocity));
	}

	protected boolean isAtVelocity(Rotation2d targetVelocity, Rotation2d velocityTolerance) {
		return MathUtil.isNear(
			targetVelocity.getRotations(),
			topFlywheelComponents.velocitySignal().getLatestValue().getRotations(),
			velocityTolerance.getRotations()
		);
	}

	protected void stop() {
		topMotor.stop();
		bottomMotor.stop();
	}

	protected void updateInputs() {
		topMotor.updateSignals(topFlywheelComponents.voltageSignal(), topFlywheelComponents.velocitySignal());
		bottomMotor.updateSignals(bottomFlywheelComponents.voltageSignal(), bottomFlywheelComponents.velocitySignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
