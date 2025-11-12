package frc.robot.subsystems.roller;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxConfiguration;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.rev.request.SparkMaxRequest;
import frc.robot.hardware.rev.request.SparkMaxRequestBuilder;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.utils.AngleUnit;

import java.util.function.Supplier;

public class SparkMaxRollerBuilder {

	public static Roller createSparkMaxMotorRoller(
		String logPath,
		int ID,
		double gearRatio,
		int currentLimiting,
		Rotation2d tolerance,
		double KP,
		double KI,
		double KD
	) {
		SparkMaxDeviceID rotorID = new SparkMaxDeviceID(ID);
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(rotorID);

		SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
			new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, gearRatio), DCMotor.getNEO(1))
		);

		BrushlessSparkMAXMotor roller = new BrushlessSparkMAXMotor(
			logPath,
			sparkMaxWrapper,
			rollerSimulation,
			new SysIdRoutine.Config()

		);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("current", sparkMaxWrapper::getOutputCurrent);
		Supplier<Double> positionSupplier = () -> sparkMaxWrapper.getEncoder().getPosition();
		SuppliedAngleSignal latencySignal = new SuppliedAngleSignal("angle", positionSupplier, AngleUnit.ROTATIONS);
		roller.applyConfiguration(configRoller(gearRatio, currentLimiting, KP, KI, KD));
		SparkMaxRequest<Double> setVoltageRequest = SparkMaxRequestBuilder.build(0.0, SparkBase.ControlType.kVoltage, 0);
		SparkMaxRequest<Rotation2d> goToAngleRequest = SparkMaxRequestBuilder
			.build(Rotation2d.fromRotations(0), SparkBase.ControlType.kPosition, 0);
		return new Roller(logPath, roller, voltageSignal, currentSignal, latencySignal, setVoltageRequest, goToAngleRequest, tolerance);
	}

	private static SparkMaxConfiguration configRoller(double gearRatio, int currentLimiting, double KP, double KI, double KD) {
		SparkMaxConfiguration configs = new SparkMaxConfiguration();
		configs.getSparkMaxConfig().smartCurrentLimit(currentLimiting);
		configs.getSparkMaxConfig().encoder.positionConversionFactor(gearRatio);
		configs.getSparkMaxConfig().encoder.velocityConversionFactor(gearRatio);
		configs.getSparkMaxConfig().closedLoop.pid(KP, KI, KD);
		return configs;
	}

}
