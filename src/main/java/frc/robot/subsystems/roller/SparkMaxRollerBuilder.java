package frc.robot.subsystems.roller;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
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

	private static SimpleMotorSimulation buildSimulation(double gearRatio, double momentOfInertia) {
		return new SimpleMotorSimulation(
			new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), momentOfInertia, gearRatio), DCMotor.getNEO(1))
		);
	}

	private static Roller buildRoller(
		String logPath,
		SparkMaxWrapper sparkMaxWrapper,
		boolean inverted,
		double gearRatio,
		int currentLimit,
		double momentOfInertia
	) {
		SimpleMotorSimulation rollerSimulation = buildSimulation(gearRatio, momentOfInertia);

		BrushlessSparkMAXMotor roller = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, rollerSimulation, new SysIdRoutine.Config());

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("current", sparkMaxWrapper::getOutputCurrent);
		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal(
			"position",
			() -> sparkMaxWrapper.getEncoder().getPosition(),
			AngleUnit.ROTATIONS
		);

		roller.applyConfiguration(buildConfiguration(inverted, gearRatio, currentLimit));

		SparkMaxRequest<Double> voltageRequest = SparkMaxRequestBuilder.build(0.0, SparkBase.ControlType.kVoltage, ClosedLoopSlot.kSlot0);

		return new Roller(logPath, roller, voltageSignal, currentSignal, positionSignal, voltageRequest);
	}

	public static Roller build(
		String logPath,
		SparkMaxDeviceID id,
		boolean inverted,
		double gearRatio,
		int currentLimit,
		double momentOfInertia
	) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(id);
		return buildRoller(logPath, sparkMaxWrapper, inverted, gearRatio, currentLimit, momentOfInertia);
	}

	public static Pair<Roller, IDigitalInput> buildWithDigitalInput(
		String logPath,
		SparkMaxDeviceID id,
		boolean inverted,
		double gearRatio,
		int currentLimit,
		double momentOfInertia,
		String digitalInputName,
		double debounceTime,
		boolean isForwardLimitSwitch,
		boolean isLimitSwitchInverted
	) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(id);

		IDigitalInput digitalInput;
		if (Robot.ROBOT_TYPE.isSimulation()) {
			digitalInput = new ChooserDigitalInput(digitalInputName);
		} else {
			if (isForwardLimitSwitch) {
				digitalInput = new SuppliedDigitalInput(
					() -> sparkMaxWrapper.getForwardLimitSwitch().isPressed(),
					new Debouncer(debounceTime),
					isLimitSwitchInverted
				);
			} else {
				digitalInput = new SuppliedDigitalInput(
					() -> sparkMaxWrapper.getReverseLimitSwitch().isPressed(),
					new Debouncer(debounceTime),
					isLimitSwitchInverted
				);
			}
		}
		return new Pair<>(buildRoller(logPath, sparkMaxWrapper, inverted, gearRatio, currentLimit, momentOfInertia), digitalInput);
	}

	private static SparkMaxConfiguration buildConfiguration(boolean inverted, double gearRatio, int currentLimit) {
		SparkMaxConfiguration configs = new SparkMaxConfiguration();
		configs.getSparkMaxConfig().smartCurrentLimit(currentLimit);
		configs.getSparkMaxConfig().inverted(inverted);
		configs.getSparkMaxConfig().encoder.positionConversionFactor(1 / gearRatio);
		configs.getSparkMaxConfig().encoder.velocityConversionFactor(1 / gearRatio);
		return configs;
	}

}
