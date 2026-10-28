package frc.robot.subsystems.moduleAlon;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.AlonFX;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class ModuleAlon extends GBSubsystem {

	private final AlonFX linear;
	private final AlonFX steer;
	private final String logpath;
	private final ModuleCommandsBuilder commandsBuilder;

	public ModuleAlon(
		int steerID,
		int linearID,
		double steerGearRatio,
		double linearGearRatio,
		double steerKp,
		double linearKp,
		double steerMomentOfInertiaKgM2,
		double linearMomentOfInertiaKgM2,
		CANBus canBus,
		String logPath
	) {
		super(logPath);
		this.steer = new AlonFX(steerID, canBus, logPath + "/steer", steerGearRatio, steerKp, steerMomentOfInertiaKgM2);
		this.linear = new AlonFX(linearID, canBus, logPath + "/drive", linearGearRatio, linearKp, linearMomentOfInertiaKgM2);
		this.logpath = logPath;

		commandsBuilder = new ModuleCommandsBuilder(this, logPath + "/commandsBuilder");
		Command defaultCommand = new InstantCommand(() -> {
			stop();
		});
		defaultCommand.addRequirements(this);
		setDefaultCommand(defaultCommand);
	}

	public ModuleCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
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

	public void linearPower(double power) {
		linear.setPower(power);
	}

	public Rotation2d getSteerPosition() {
		return steer.getPosition();
	}

	public Rotation2d getLinearPosition() {
		return linear.getPosition();
	}

	public Rotation2d getLinearVelocity() {
		return linear.getVelocity();
	}

	public void setLinearNeutral(boolean brake) {
		linear.setBrake(brake);
	}

	public void setSteerNeutral(boolean brake) {
		steer.setBrake(brake);
	}

	public void setSteerPosition(Rotation2d position) {
		steer.setPosition(position);
	}

	public void setLinearPosition(Rotation2d position) {
		linear.setPosition(position);
	}

	public void logAll() {
		Logger.recordOutput(logpath + "/linearMotorVelocity", getLinearVelocity());
		Logger.recordOutput(logpath + "/steerAngle", getSteerPosition());
		Logger.recordOutput(logpath + "/steerTarget", steer.getPIDTarget());
		Logger.recordOutput(logpath + "/linearTarget", linear.getPIDTarget());
		steer.logAll();
		linear.logAll();
	}

	public void stop() {
		linear.stop();
		steer.stop();
	}

	public void updateSimulation(){
		linear.updateSimulation();
		steer.updateSimulation();
	}

}
