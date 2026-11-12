package frc.robot.subsystems.tal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.AngleUnit;
import frc.utils.battery.BatteryUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class GBModuleTal extends GBSubsystem {

	private final TalonFXMotor driveMotor;
	private final TalonFXMotor steerMotor;


	private final TalonFXConfiguration steerConfigs;
	private final TalonFXConfiguration driveConfigs;


	private InvertedValue steerDirection;
	private InvertedValue driveDirection;
	private boolean driveBrake;
	private boolean steerBrake;

	private Supplier<Double> targetDrivePower;
	private Supplier<Rotation2d> targetSteerAngle;

	private ModulePowers modulePowers;
	private final PositionVoltage request;

	private GBModuleTalCommandBuilder commandBuilder = new GBModuleTalCommandBuilder(this);

	private SimpleMotorSimulation steerSimulation;
	private SimpleMotorSimulation driveSimulation;
	private TalonFXSimState steerMotorSimState;
	private TalonFXSimState driveMotorSimState;

	Phoenix6AngleSignal drivePositionSignal;
	Phoenix6AngleSignal steerPositionSignal;
	Phoenix6DoubleSignal driveVoltageSignal;


	public GBModuleTal(String logPath, int driveID, int steerID) {
		super(logPath);
		this.driveMotor = new TalonFXMotor(logPath + "/driveMotor", new Phoenix6DeviceID(driveID), new SysIdRoutine.Config());
		this.steerMotor = new TalonFXMotor(logPath + "/steerMotor", new Phoenix6DeviceID(steerID), new SysIdRoutine.Config());
		this.steerConfigs = new TalonFXConfiguration();
		this.driveConfigs = new TalonFXConfiguration();
		this.steerBrake = false;
		this.driveBrake = false;
		configureSteerMotor();
		configureDriveMotor();
		this.targetSteerAngle = () -> new Rotation2d(0);
		this.targetDrivePower = () -> (Double) 0.0;
		this.setAmpereLimit(ModuleTalConstants.ampereLimit);
		this.commandBuilder = new GBModuleTalCommandBuilder(this);
		this.request = new PositionVoltage(0).withSlot(0);


		this.drivePositionSignal = Phoenix6SignalBuilder
			.build(this.driveMotor.getDevice().getPosition(), ModuleTalConstants.signalUpdateFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		this.steerPositionSignal = Phoenix6SignalBuilder
			.build(this.steerMotor.getDevice().getPosition(), ModuleTalConstants.signalUpdateFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		this.driveVoltageSignal = Phoenix6SignalBuilder
			.build(this.driveMotor.getDevice().getMotorVoltage(), ModuleTalConstants.signalUpdateFrequency, BusChain.ROBORIO);

		if (Robot.ROBOT_TYPE.isSimulation()) {
			this.driveSimulation = new SimpleMotorSimulation(
				new DCMotorSim(
					LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.0000001, ModuleTalConstants.driveGearRatio),
					DCMotor.getKrakenX60(1)
				)
			);
			this.steerSimulation = new SimpleMotorSimulation(
				new DCMotorSim(
					LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.0000001, ModuleTalConstants.steerGearRatio),
					DCMotor.getKrakenX60(1)
				)
			);

			this.steerMotorSimState = steerMotor.getDevice().getSimState();
			steerMotorSimState.setSupplyVoltage(BatteryUtil.DEFAULT_VOLTAGE);

			this.driveMotorSimState = driveMotor.getDevice().getSimState();
			driveMotorSimState.setSupplyVoltage(BatteryUtil.DEFAULT_VOLTAGE);
		}
	}

	private void configureSteerMotor() {
		this.steerConfigs.Slot0.kP = ModuleTalConstants.kPSteer;
		this.steerConfigs.Slot0.kI = ModuleTalConstants.kISteer;
		this.steerConfigs.Slot0.kD = ModuleTalConstants.kDSteer;
		this.steerConfigs.Feedback.SensorToMechanismRatio = ModuleTalConstants.steerGearRatio;
		this.steerMotor.applyConfiguration(this.steerConfigs);
	}

	private void configureDriveMotor() {
		this.driveConfigs.Slot0.kP = ModuleTalConstants.kPDrive;
		this.driveConfigs.Slot0.kI = ModuleTalConstants.kIDrive;
		this.driveConfigs.Slot0.kD = ModuleTalConstants.kDDrive;
		this.driveConfigs.Feedback.SensorToMechanismRatio = ModuleTalConstants.driveGearRatio;
		this.driveMotor.applyConfiguration(this.driveConfigs);
	}

	public void setDriveAmpereLimit(double ampereLimit) {
		this.driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		this.driveConfigs.CurrentLimits.withStatorCurrentLimit(ampereLimit);
		this.driveMotor.applyConfiguration(this.driveConfigs);
	}

	public void setSteerAmpereLimit(double ampereLimit) {
		this.steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		this.steerConfigs.CurrentLimits.withStatorCurrentLimit(ampereLimit);
		this.steerMotor.applyConfiguration(this.steerConfigs);
	}

	public void setAmpereLimit(double ampereLimit) {
		this.setSteerAmpereLimit(ampereLimit);
		this.setDriveAmpereLimit(ampereLimit);
	}

	public GBModuleTalCommandBuilder getCommandBuilder() {
		return this.commandBuilder;
	}

	public void changeDriveBrake() {
		driveBrake = !driveBrake;
		this.driveMotor.setBrake(driveBrake);
	}

	public void changeSteerBrake() {
		steerBrake = !steerBrake;
		this.steerMotor.setBrake(steerBrake);
	}

	protected void invertDriveDirection() {
		this.driveDirection = this.driveDirection == InvertedValue.Clockwise_Positive
			? InvertedValue.CounterClockwise_Positive
			: InvertedValue.Clockwise_Positive;
		this.driveConfigs.MotorOutput.withInverted(this.driveDirection);
		this.driveMotor.applyConfiguration(this.driveConfigs);
	}

	protected void invertSteerDirection() {
		this.steerDirection = this.steerDirection == InvertedValue.Clockwise_Positive
			? InvertedValue.CounterClockwise_Positive
			: InvertedValue.Clockwise_Positive;
		this.steerConfigs.MotorOutput.withInverted(this.steerDirection);
		this.steerMotor.applyConfiguration(this.steerConfigs);
	}

	public void invertMotors() {
		this.invertDriveDirection();
		this.invertSteerDirection();
	}

	public void changeMotorsBrake() {
		this.changeDriveBrake();
		this.changeSteerBrake();
	}

	public void drive(Rotation2d angle, double power) {
		steerToUnrestrictedAngle(angle);
		setDrivePower(power);
	}


	public void steerToUnrestrictedAngle(Rotation2d angle) {
//		Rotation2d currentAngle = this.getSteerAngle();
//		Logger.recordOutput(getLogPath() + "/currentAngleSteer", currentAngle);
//		if (Math.signum(currentAngle.getDegrees()) != Math.signum(angle.getDegrees()) && angle.getDegrees()!=0) {
//			double newAngle = currentAngle.getDegrees() + (360 - currentAngle.getDegrees() - angle.getDegrees());
//			Logger.recordOutput(getLogPath() + "/newAngle", currentAngle);
//			angle = Rotation2d.fromDegrees(newAngle);
//			this.steerMotor.applyRequest(Phoenix6RequestBuilder.build(request.withPosition(angle.getRotations()), 0.5, false));
//            this.steerMotor.getDevice().setPosition(angle.getRotations() * -1);
//		} else {
//			this.steerMotor.applyRequest(Phoenix6RequestBuilder.build(request.withPosition(angle.getRotations()), 0.5, false));
//		}
//		Rotation2d currentAngle = this.getSteerAngle();
//		Logger.recordOutput(getLogPath() + "/halfRotations", this.halfRotationsMade);
//		if (Math.signum(currentAngle.getDegrees()) != Math.signum(angle.getDegrees()) && angle.getDegrees() != 0) {
//			if (!theWorstFlag && angle.getDegrees()!=0) {
//				if (Math.signum(currentAngle.getDegrees()) > Math.signum(angle.getDegrees())) {
//					this.halfRotationsMade++;
//				} else {
//					this.halfRotationsMade--;
//				}
//			}
//			theWorstFlag = true;
//
//			angle = Rotation2d.fromDegrees(180 * this.halfRotationsMade + (Math.signum(this.halfRotationsMade) * 180) + angle.getDegrees());
//			Logger.recordOutput(getLogPath() + "/theAngle", angle);
//			this.steerMotor.applyRequest(Phoenix6RequestBuilder.build(request.withPosition(angle.getRotations()), 0.5, false));
//		} else {
//			theWorstFlag = false;


		Rotation2d currentAngle = this.getSteerAngle();
		double diff = angle.getDegrees() - currentAngle.getDegrees();
		// -170 - 225 = -395
		diff = diff - 360.0 * Math.floor((diff + 180) / 360);
		// -395 - 360*floor(-215)/360 = -45


		this.steerMotor.applyRequest(
			Phoenix6RequestBuilder
				.build(request.withPosition(currentAngle.getRotations() + Rotation2d.fromDegrees(diff).getRotations()), 0.5, false)
		);
	}

	public void steerToAngle(Rotation2d angle) {
		this.steerMotor.applyRequest(Phoenix6RequestBuilder.build(request.withPosition(angle.getRotations()), 0.5, false));
	}


	public double getDriveVoltage() {
		return this.driveVoltageSignal.getAndUpdateValue();
	}

	public void setDriveVoltage(double voltage) {
		this.driveMotor.getDevice().setVoltage(voltage);
	}

	public void setDrivePower(double power) {
		this.driveMotor.setPower(power);
	}


	public void stop() {
		this.driveMotor.stop();
		this.steerMotor.stop();
	}

	public Rotation2d getSteerAngle() {
		return this.steerPositionSignal.getAndUpdateValue();
	}

	public double getDrivePower() {
		return this.driveMotor.getDevice().get();
	}

	public void setModulePowers(ModulePowers modulePowers) {
		this.modulePowers = modulePowers;
	}


	public boolean isDriveAtPower(double power) {
		return this.getDrivePower() == power;
	}

	public boolean isSteerAtAngle(Rotation2d angle) {
		return this.getSteerAngle() == angle;
	}

	public Supplier<Double> getTargetDrivePower() {
		return targetDrivePower;
	}

	public Rotation2d getDrivePosition() {
		return this.drivePositionSignal.getAndUpdateValue();
	}

	@Override
	protected void subsystemPeriodic() {
		if (Robot.ROBOT_TYPE.isSimulation()) {
			updateSteerSim();
			updateDriveSim();
		}
		driveMotor.updateInputs(drivePositionSignal, driveVoltageSignal);
		steerMotor.updateInputs(steerPositionSignal);
		Logger.recordOutput(getLogPath() + "/currentDriveSpeed", getDrivePower());
		Logger.recordOutput(getLogPath() + "/targetDrivePower", getTargetDrivePower().get());
		Logger.recordOutput(getLogPath() + "/targetWheelAngle", this.targetSteerAngle.get());
	}

	public void update() {
		this.targetDrivePower = () -> (Double) Math.sqrt(Math.pow(this.modulePowers.xPower, 2) + Math.pow(this.modulePowers.yPower, 2));
		this.targetSteerAngle = this::calculateAngle;
	}

	public Rotation2d getTargetSteerAngle() {
		return targetSteerAngle.get();
	}

	protected Rotation2d calculateAngle() {
		return new Rotation2d(Math.atan2(this.modulePowers.yPower, this.modulePowers.xPower));
	}

	protected void updateSteerSim() {
		steerSimulation.setInputVoltage(steerMotorSimState.getMotorVoltage());
		steerSimulation.updateMotor();
		steerMotorSimState.setRawRotorPosition(steerSimulation.getRotorPosition().getRotations());
		steerMotorSimState.setRotorAcceleration(steerSimulation.getRotorVelocityRPS().getRotations());
	}

	protected void updateDriveSim() {
		driveSimulation.setInputVoltage(driveMotorSimState.getMotorVoltage());
		driveSimulation.updateMotor();
		driveMotorSimState.setRawRotorPosition(driveSimulation.getRotorPosition().getRotations());
		driveMotorSimState.setRotorAcceleration(driveSimulation.getRotorVelocityRPS().getRotations());
	}

}
