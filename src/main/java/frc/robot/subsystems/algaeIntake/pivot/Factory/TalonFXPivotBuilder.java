package frc.robot.subsystems.algaeIntake.pivot.Factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.algaeIntake.pivot.Pivot;
import frc.robot.subsystems.algaeIntake.pivot.PivotConstants;
import frc.utils.math.AngleUnit;

public class TalonFXPivotBuilder {

	private static final int NUMBER_OF_MOTORS = 1;
	private static final double GEAR_RATIO = 1 / 1;
	private static final boolean SIMULATE_GRAVITY = false;
	private static final double DEFAULT_ARBITRARY_FEED_FORWARD = 0;
	private static final boolean ENABLE_FOC = false;
	private static final double REAL_KP = 1;
	private static final double REAL_KG = 0;
	private static final double SIMULATION_KP = 1;
	private static final double SIMULATION_KG = 0;

	public static TalonFXMotor generateMotor(String logPath) {
		SingleJointedArmSimulation sim = new SingleJointedArmSimulation(
			new SingleJointedArmSim(
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS),
					SingleJointedArmSim.estimateMOI(PivotConstants.LENGTH_METERS, PivotConstants.MASS_KG),
					GEAR_RATIO
				),
				DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS),
				GEAR_RATIO,
				PivotConstants.LENGTH_METERS,
				PivotConstants.MIN_POSITION.getRadians(),
				PivotConstants.MAX_POSITION.getRadians(),
				SIMULATE_GRAVITY,
				PivotConstants.STARTING_POSITION.getRadians()
			),
			GEAR_RATIO
		);

		return new TalonFXMotor(logPath + "/Motor", IDs.TalonFXIDs.PIVOT, new SysIdRoutine.Config(), sim);
	}

	private static TalonFXConfiguration generateMotorConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		switch (Robot.ROBOT_TYPE) {
			case REAL -> {
				config.Slot0.kP = REAL_KP;
				config.Slot0.kG = REAL_KG;
			}
			case SIMULATION -> {
				config.Slot0.kP = SIMULATION_KP;
				config.Slot0.kG = SIMULATION_KG;
			}
		}

		config.MotorOutput.Inverted = PivotConstants.IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.FORWARD_LIMIT.getRotations();
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.BACKWARD_LIMIT.getRotations();
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

		config.Feedback.RotorToSensorRatio = GEAR_RATIO;

		return config;
	}

	public static Pivot generate(String logPath) {
		TalonFXMotor pivot = generateMotor(logPath);
		pivot.applyConfiguration(generateMotorConfig());

		IRequest<Rotation2d> positionRequest = Phoenix6RequestBuilder
			.build(new PositionVoltage(PivotConstants.STARTING_POSITION.getRotations()), DEFAULT_ARBITRARY_FEED_FORWARD, ENABLE_FOC);

		InputSignal<Rotation2d> positionSignal = Phoenix6SignalBuilder
			.build(pivot.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		InputSignal<Double> voltageSignal = Phoenix6SignalBuilder
			.build(pivot.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);

		return new Pivot(logPath, pivot, positionRequest, positionSignal, voltageSignal);
	}

}
