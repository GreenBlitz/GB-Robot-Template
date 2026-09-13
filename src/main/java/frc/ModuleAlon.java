package frc;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class ModuleAlon extends GBSubsystem {

	private final AlonFX linear;
	private final AlonFX steer;
	private final String LOGPATH;
	private final ModulesCommandBuilder commandBuilder;

	public ModuleAlon(int steerID, int linearID, double steerGearRatio, double linearGearRatio, CANBus canBus, String logPath) {
		super(logPath);
		this.linear = new AlonFX(steerID, canBus, logPath + "/steer", steerGearRatio,2);
		this.steer = new AlonFX(linearID, canBus, logPath + "/drive", linearGearRatio,2);
		this.LOGPATH = logPath;
		super.setDefaultCommand(new InstantCommand(() -> stop()));
		commandBuilder = new ModulesCommandBuilder(this);
	}

	public ModulesCommandBuilder getCommandBuilder(){
		return commandBuilder;
	}

	public void invertLinear() {
		linear.invertMotor();
	}

	public void invertSteer() {
		steer.invertMotor();
	}

	public void steerToPosition(double angleRadians) {
		steer.driveToPosition(angleRadians);
	}

	public void linearStablePower(double power) {
		linear.setPower(power);
	}

	public void linearSetPower(double power) {
		linear.setPower(power);
	}

	public Rotation2d getSteerAngle() {
		return steer.getPosition();
	}

	public Rotation2d getLinearAngle(){
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

	public void setSteerPosition(double position) {
		steer.setPosition(position);
	}

	public void setLinearPosition(double position) {
		linear.setPosition(position);
	}

	public void logAll() {
		Logger.recordOutput(LOGPATH + "/linearMotorVelocity", getLinearVelocity());
		Logger.recordOutput(LOGPATH + "/steerAngle", getSteerAngle());
		Logger.recordOutput(LOGPATH + "/steerTarget", steer.getPIDTarget());
		Logger.recordOutput(LOGPATH + "/linearTarget", linear.getPIDTarget());
	}

	public void stop() {
		linear.stopMotor();
		steer.stopMotor();
	}


}
