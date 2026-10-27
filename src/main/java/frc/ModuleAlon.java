package frc;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.hardware.interfaces.IMotor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;


public class ModuleAlon extends GBSubsystem {

	private final AlonFX linear;
	private final AlonFX steer;
	private final String logpath;
	private final ModuleCommandsBuilder commandBuilder;

	public ModuleAlon(
		int steerID,
		int linearID,
		double steerGearRatio,
		double linearGearRatio,
		double steerKp,
		double linearKp,
		CANBus canBus,
		String logPath
	) {
		super(logPath);
		this.steer = new AlonFX(steerID, canBus, logPath + "/steer", steerGearRatio, steerKp);
		this.linear = new AlonFX(linearID, canBus, logPath + "/drive", linearGearRatio, linearKp);
		this.logpath = logPath;

		commandBuilder = new ModuleCommandsBuilder(this, logPath + "/commandBuilder");
		Command defaultCommand = new InstantCommand(() -> {
			stop();
		});
		defaultCommand.addRequirements(this);
		setDefaultCommand(defaultCommand);
	}

	public ModuleCommandsBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public void invertLinear() {
		linear.invertMotor();
	}

	public void invertSteer() {
		steer.invertMotor();
	}

	public void steerToPosition(Rotation2d angleRadians) {
		steer.driveToPosition(angleRadians);
	}

	public void linearSetPower(double power) {
		linear.setPower(power);
	}

	public Rotation2d getSteerAngle() {
		return steer.getPosition();
	}

	public Rotation2d getLinearAngle() {
		return linear.getPosition();
	}

	public Rotation2d getLinearVelocity() {
		return linear.getVelocity();
	}

	public void setLinearNeutral(NeutralModeValue neutralMode) {
		linear.setNeutralMode(neutralMode);
	}

	public void setSteerNeutral(NeutralModeValue neutralMode) {
		steer.setNeutralMode(neutralMode);
	}

	public void setSteerPosition(Rotation2d position) {
		steer.setPosition(position);
	}

	public void setLinearPosition(Rotation2d position) {
		linear.setPosition(position);
	}

	public void logAll() {
		Logger.recordOutput(logpath + "/linearMotorVelocity", getLinearVelocity());
		Logger.recordOutput(logpath + "/steerAngle", getSteerAngle().getDegrees());
		Logger.recordOutput(logpath + "/steerTarget", steer.getPIDTarget());
		Logger.recordOutput(logpath + "/linearTarget", linear.getPIDTarget());
		steer.logAll();
		linear.logAll();
	}

	public void stop() {
		linear.stopMotor();
		steer.stopMotor();
	}


}
