package frc.robot.subsystems.elbow.factory;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxDoubleRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.elbow.ElbowStuff;
import frc.utils.AngleUnit;

import java.util.function.Function;

public class RealElbowConstants {

	private static final double KS = 0.15;
	private static final double KG = 0.2;
	private static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(KS, KG, 0);

	private static final int POSITION_PID_SLOT = 0;

	private static SparkMaxAngleRequest generatePositionRequest() {
		Function<CANSparkMax, Double> feedforwardCalculation = canSparkMax -> FEEDFORWARD
			.calculate(canSparkMax.getEncoder().getPosition(), canSparkMax.getEncoder().getVelocity());
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
		motor.setInverted(true);
		motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) ElbowConstants.FORWARD_LIMIT.getRotations());
		motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
		motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) ElbowConstants.BACKWARD_LIMIT.getRotations());
		motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
		motor.setSmartCurrentLimit(40);
		motor.getPIDController().setP(5.5, POSITION_PID_SLOT);
		motor.getPIDController().setD(0.5, POSITION_PID_SLOT);
		motor.getEncoder().setPositionConversionFactor(ElbowConstants.GEAR_RATIO);
		motor.getEncoder().setPositionConversionFactor(ElbowConstants.GEAR_RATIO);
	}

	protected static ElbowStuff generateElbowStuff(String logPath) {
		SparkMaxWrapper motor = new SparkMaxWrapper(IDs.CANSparkMAXs.ELBOW);
		configMotor(motor);

		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", () -> motor.getEncoder().getPosition(), AngleUnit.ROTATIONS);
		SuppliedAngleSignal velocitySignal = new SuppliedAngleSignal("velocity", () -> motor.getEncoder().getVelocity(), AngleUnit.ROTATIONS);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("output current", motor::getOutputCurrent);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", () -> motor.getAppliedOutput() * motor.getBusVoltage());

		BrushlessSparkMAXMotor elbow = new BrushlessSparkMAXMotor(logPath, motor, new SysIdRoutine.Config());
		return new ElbowStuff(
			logPath,
			elbow,
			generatePositionRequest(),
			generateVoltageRequest(),
			positionSignal,
			velocitySignal,
			currentSignal,
			voltageSignal
		);
	}

}
