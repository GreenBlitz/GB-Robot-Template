package frc.robot.subsystems.roller;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
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

public class SparkMaxRollerBuilder {

	private static SparkMaxWrapper generateSparkMaxWrapper(int id){
		SparkMaxDeviceID ID = new SparkMaxDeviceID(id);
		return new SparkMaxWrapper(ID);
	}
	private static SimpleMotorSimulation generateSimulation(double gearRatio){
		return new SimpleMotorSimulation(
				new DCMotorSim(
						LinearSystemId
								.createDCMotorSystem(DCMotor.getNEO(RollerConstants.NUMBER_OF_MOTORS), RollerConstants.MOMENT_OF_INERTIA, gearRatio),
						DCMotor.getNEO(RollerConstants.NUMBER_OF_MOTORS)
				)
		);
	}
	public static Roller generate(
		String logPath,
		int id,
		double gearRatio,
		int currentLimit,
		Rotation2d tolerance,
		double kP,
		double kI,
		double kD
	) {
		SparkMaxWrapper sparkMaxWrapper = generateSparkMaxWrapper(id);

		SimpleMotorSimulation rollerSimulation = generateSimulation(gearRatio);

		BrushlessSparkMAXMotor roller = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, rollerSimulation, new SysIdRoutine.Config());

		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal(
			"position",
			() -> sparkMaxWrapper.getEncoder().getPosition(),
			AngleUnit.ROTATIONS
		);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("current", sparkMaxWrapper::getOutputCurrent);

		roller.applyConfiguration(configRoller(gearRatio, currentLimit, kP, kI, kD));

		SparkMaxRequest<Double> voltageRequest = SparkMaxRequestBuilder.build(0.0, SparkBase.ControlType.kVoltage, 0);
		SparkMaxRequest<Rotation2d> angleRequest = SparkMaxRequestBuilder.build(Rotation2d.fromRotations(0), SparkBase.ControlType.kPosition, 0);

		return new Roller(logPath, roller, voltageSignal, positionSignal, currentSignal, voltageRequest, angleRequest, tolerance);
	}

	public static Pair<Roller, IDigitalInput> generateWithDigitalInput(
		String logPath,
		int id,
		double gearRatio,
		int currentLimit,
		Rotation2d tolerance,
		double kP,
		double kI,
		double kD,
		String digitalInputName,
		int channel,
		double debounceTime,
		boolean isForwardLimitSwitch
	) {


		SparkMaxWrapper sparkMaxWrapper = generateSparkMaxWrapper(id);

		SimpleMotorSimulation rollerSimulation = generateSimulation(gearRatio);

		BrushlessSparkMAXMotor roller = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, rollerSimulation, new SysIdRoutine.Config());

		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal(
				"position",
				() -> sparkMaxWrapper.getEncoder().getPosition(),
				AngleUnit.ROTATIONS
		);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("current", sparkMaxWrapper::getOutputCurrent);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		roller.applyConfiguration(configRoller(gearRatio, currentLimit, kP, kI, kD));

		SparkMaxRequest<Rotation2d> positionRequest = SparkMaxRequestBuilder.build(Rotation2d.fromRotations(0), SparkBase.ControlType.kPosition, 0);
		SparkMaxRequest<Double> voltageRequest = SparkMaxRequestBuilder.build(0.0, SparkBase.ControlType.kVoltage, 0);

		IDigitalInput digitalInput;
		if (Robot.ROBOT_TYPE.isSimulation()) {
			digitalInput = new ChooserDigitalInput(digitalInputName);
		} else {
			if(isForwardLimitSwitch){
				digitalInput = new SuppliedDigitalInput(() -> sparkMaxWrapper.getForwardLimitSwitch().isPressed() ,new Debouncer(debounceTime));
			}
			else{
				digitalInput = new SuppliedDigitalInput(() -> sparkMaxWrapper.getForwardLimitSwitch().isPressed(),new Debouncer(debounceTime),true);
			}
		}
		return new Pair<Roller, IDigitalInput>(new Roller(logPath,roller,voltageSignal,positionSignal,currentSignal,voltageRequest,positionRequest,tolerance),digitalInput);
	}

	private static SparkMaxConfiguration configRoller(double gearRatio, int currentLimit, double kP, double kI, double kD) {
		SparkMaxConfiguration configs = new SparkMaxConfiguration();
		configs.getSparkMaxConfig().smartCurrentLimit(currentLimit);
		configs.getSparkMaxConfig().encoder.positionConversionFactor(gearRatio);
		configs.getSparkMaxConfig().encoder.velocityConversionFactor(gearRatio);
		configs.getSparkMaxConfig().closedLoop.pid(kP, kI, kD);
		return configs;
	}

}
