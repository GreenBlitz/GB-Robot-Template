package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class Flywheel extends GBSubsystem {

	private final FlywheelStuff topFlywheelStuff;
	private final FlywheelStuff bottomFlywheelStuff;
	private final ControllableMotor topMotor;
	private final ControllableMotor bottomMotor;
	private final FlywheelCommandsBuilder commandsBuilder;
	private final SysIdCalibrator sysIdCalibrator;

	public Flywheel(FlywheelStuff topFlywheelStuff, FlywheelStuff bottomFlywheelStuff, String logPath, Robot robot) {
		super(logPath);

		this.topFlywheelStuff = topFlywheelStuff;
		this.bottomFlywheelStuff = bottomFlywheelStuff;
		this.topMotor = topFlywheelStuff.motor();
		this.bottomMotor = bottomFlywheelStuff.motor();
		this.commandsBuilder = new FlywheelCommandsBuilder(this);
		this.sysIdCalibrator = new SysIdCalibrator(bottomMotor.getSysidConfigInfo(), this, this::setVoltage);

		updateInputs();
	}

	public FlywheelCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public SysIdCalibrator getSysIdCalibrator() {
		return sysIdCalibrator;
	}

	protected void setPower(double power) {
		topMotor.setPower(power);
		bottomMotor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		topMotor.applyDoubleRequest(topFlywheelStuff.voltageRequest().withSetPoint(voltage));
		bottomMotor.applyDoubleRequest(bottomFlywheelStuff.voltageRequest().withSetPoint(voltage));
	}

	protected void setTargetVelocity(Rotation2d targetVelocity) {
		topMotor.applyAngleRequest(topFlywheelStuff.velocityRequest().withSetPoint(targetVelocity));
		bottomMotor.applyAngleRequest(bottomFlywheelStuff.velocityRequest().withSetPoint(targetVelocity));
	}

	public boolean isAtVelocity(Rotation2d targetVelocity, Rotation2d velocityTolerance) {
		return MathUtil.isNear(
			targetVelocity.getRotations(),
			topFlywheelStuff.velocitySignal().getLatestValue().getRotations(),
			velocityTolerance.getRotations()
		)
			&& MathUtil.isNear(
				targetVelocity.getRotations(),
				bottomFlywheelStuff.velocitySignal().getLatestValue().getRotations(),
				velocityTolerance.getRotations()
			);
	}

	protected void stop() {
		topMotor.stop();
		bottomMotor.stop();
	}

	protected void updateInputs() {
		topMotor.updateSignals(topFlywheelStuff.voltageSignal(), topFlywheelStuff.velocitySignal());
		bottomMotor.updateSignals(bottomFlywheelStuff.voltageSignal(), bottomFlywheelStuff.velocitySignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
