package frc.robot.subsystems.elbow.factory;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxDoubleRequest;
import frc.robot.hardware.signal.cansparkmax.SparkMaxAngleSignal;
import frc.robot.hardware.signal.cansparkmax.SparkMaxDoubleSignal;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.elbow.ElbowStuff;
import frc.utils.AngleUnit;

import java.util.function.Function;

public class RealElbowConstants {

	private static final double KS = 0;
	private static final double KG = 0;
	private static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(KS, KG, 0);
	
	private static final int POSITION_PID_SLOT = 0;
	
	private static SparkMaxAngleRequest generatePositionRequest() {
		Function<CANSparkMax, Double> feedforwardCalculation = canSparkMax -> FEEDFORWARD.calculate(
				canSparkMax.getEncoder().getPosition(),
				canSparkMax.getEncoder().getVelocity()
		);
		return new SparkMaxAngleRequest(
				new Rotation2d(),
				SparkMaxAngleRequest.SparkAngleRequestType.POSITION,
				POSITION_PID_SLOT,
				feedforwardCalculation
		);
	}
	
	private static SparkMaxDoubleRequest generateVoltageRequest() {
		return new SparkMaxDoubleRequest(0, SparkMaxDoubleRequest.SparkDoubleRequestType.VOLTAGE, 0);
	}
	
	private static void configMotor(CANSparkMax motor) {
		motor.setInverted(false);
		motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		motor.setSoftLimit(
				CANSparkBase.SoftLimitDirection.kForward,
				(float) ElbowConstants.FORWARD_LIMIT.getRotations()
		);
		motor.setSoftLimit(
				CANSparkBase.SoftLimitDirection.kReverse,
				(float) ElbowConstants.BACKWARD_LIMIT.getRotations()
		);
		motor.setSmartCurrentLimit(40);
		motor.getPIDController().setP(0, POSITION_PID_SLOT);
		motor.getEncoder().setPositionConversionFactor(ElbowConstants.GEAR_RATIO);
		motor.getEncoder().setPositionConversionFactor(ElbowConstants.GEAR_RATIO);
	}
	
 	protected static ElbowStuff generateElbowStuff(String logPath) {
		SparkMaxWrapper motor = new SparkMaxWrapper(IDs.CANSparkMAXIDs.ELBOW_MOTOR);
		configMotor(motor);
		
		SparkMaxAngleSignal positionSignal = new SparkMaxAngleSignal("position", () -> motor.getEncoder().getPosition(), AngleUnit.ROTATIONS);
		SparkMaxAngleSignal velocitySignal = new SparkMaxAngleSignal("position", () -> motor.getEncoder().getVelocity(), AngleUnit.ROTATIONS);
		SparkMaxDoubleSignal currentSignal = new SparkMaxDoubleSignal("output current", motor::getOutputCurrent);
		SparkMaxDoubleSignal voltageSignal = new SparkMaxDoubleSignal("voltage", () -> motor.getAppliedOutput() * motor.getBusVoltage());
		
		BrushlessSparkMAXMotor elbow = new BrushlessSparkMAXMotor(logPath, motor, new SysIdRoutine.Config());
		return new ElbowStuff(elbow, generatePositionRequest(), generateVoltageRequest(), positionSignal, velocitySignal, currentSignal, voltageSignal);
 	}

}
