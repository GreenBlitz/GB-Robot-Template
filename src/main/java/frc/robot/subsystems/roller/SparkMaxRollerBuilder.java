package frc.robot.subsystems.roller;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
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
		SparkMaxDeviceID ID = new SparkMaxDeviceID(id);
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(ID);

		SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId
					.createDCMotorSystem(DCMotor.getNEO(RollerConstants.NUMBER_OF_MOTORS), RollerConstants.MOMENT_OF_INERTIA, gearRatio),
				DCMotor.getNEO(RollerConstants.NUMBER_OF_MOTORS)
			)
		);

		BrushlessSparkMAXMotor roller = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, rollerSimulation, new SysIdRoutine.Config());

		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", () -> sparkMaxWrapper.getEncoder().getPosition(), AngleUnit.ROTATIONS);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("current", sparkMaxWrapper::getOutputCurrent);

		roller.applyConfiguration(configRoller(gearRatio, currentLimit, kP, kI, kD));

		SparkMaxRequest<Double> voltageRequest = SparkMaxRequestBuilder.build(0.0, SparkBase.ControlType.kVoltage, 0);
		SparkMaxRequest<Rotation2d> angleRequest = SparkMaxRequestBuilder.build(Rotation2d.fromRotations(0), SparkBase.ControlType.kPosition, 0);

		return new Roller(logPath, roller, voltageSignal, positionSignal, currentSignal, voltageRequest, angleRequest, tolerance);
	}

	public static Pair<Roller, IDigitalInput> generateWithDigitalInput(String logPath, int id, double gearRatio, int currentLimit, Rotation2d tolerance, double kP, double kI, double kD,String digitalInputName,int channel,double debounceTime){
		IDigitalInput digitalInput;
		if (Robot.ROBOT_TYPE.isSimulation()){
			digitalInput = new ChooserDigitalInput(digitalInputName);
		}
		else {
			digitalInput = new ChanneledDigitalInput(new DigitalInput(channel), new Debouncer(debounceTime));
		}
		return new Pair<>(generate(logPath,id,gearRatio,currentLimit,tolerance,kP,kI,kD),digitalInput);
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
