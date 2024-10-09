package frc.robot.subsystems.intake.pivot.factory;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.intake.pivot.PivotConstants;
import frc.robot.subsystems.intake.pivot.PivotStuff;
import frc.utils.AngleUnit;

import java.util.function.Function;

public class RealPivotConstants {

	private static final double GEAR_RATIO = 1;

	public static final int POSITION_PID_SLOT = 0;

	private static final ArmFeedforward FEEDFORWARD_CALCULATOR = new ArmFeedforward(0, 0, 0);

	//@formatter:off
	private static final Function<Rotation2d, Double> FEEDFORWARD_FUNCTION =
		position -> FEEDFORWARD_CALCULATOR.calculate(position.getRadians(), 0);
	//@formatter:on

	private static void configMotor(SparkMaxWrapper sparkMaxWrapper) {
		sparkMaxWrapper.getEncoder().setPositionConversionFactor(GEAR_RATIO);
		sparkMaxWrapper.getEncoder().setVelocityConversionFactor(GEAR_RATIO);
		sparkMaxWrapper.getPIDController().setP(1);
		sparkMaxWrapper.getPIDController().setI(0);
		sparkMaxWrapper.getPIDController().setD(0);
		sparkMaxWrapper.setSmartCurrentLimit(30);
		sparkMaxWrapper.setIdleMode(CANSparkBase.IdleMode.kCoast);
		sparkMaxWrapper.setInverted(false);
	}

	static PivotStuff generatePivotStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.PIVOT);
		configMotor(sparkMaxWrapper);

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(PivotConstants.LOG_PATH, sparkMaxWrapper, new SysIdRoutine.Config());

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);
		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", sparkMaxWrapper.getEncoder()::getPosition, AngleUnit.ROTATIONS);

		SparkMaxAngleRequest positionRequest = new SparkMaxAngleRequest(
			positionSignal.getLatestValue(),
			SparkMaxAngleRequest.SparkAngleRequestType.POSITION,
			POSITION_PID_SLOT,
			FEEDFORWARD_FUNCTION
		);

		return new PivotStuff(logPath, motor, voltageSignal, positionSignal, positionRequest);
	}

}
