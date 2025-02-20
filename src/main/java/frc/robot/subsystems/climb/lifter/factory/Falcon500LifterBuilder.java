package frc.robot.subsystems.climb.lifter.factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.climb.lifter.Lifter;
import frc.robot.subsystems.climb.lifter.LifterConstants;
import frc.utils.math.AngleUnit;

public class Falcon500LifterBuilder {

	private static final int NUMBER_OF_MOTORS = 1;

	private static final double CURRENT_LIMIT = 40;

	private static final boolean SET_BRAKE = true;
	private static final boolean INVERTED = true;
	private static final double MOMENT_OF_INERTIA = 0.001;

	private static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(200);

	private static TalonFXConfiguration generateMotorConfiguration() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.Feedback.SensorToMechanismRatio = LifterConstants.GEAR_RATIO;
		configuration.MotorOutput.Inverted = INVERTED ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
		configuration.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;

		return configuration;
	}

	protected static Lifter createLifter(String logPath) {
		SingleJointedArmSimulation simulation = new SingleJointedArmSimulation(
			new SingleJointedArmSim(
				DCMotor.getFalcon500Foc(NUMBER_OF_MOTORS),
				LifterConstants.GEAR_RATIO,
				MOMENT_OF_INERTIA,
				LifterConstants.LIFTER_LENGTH_METERS,
				LifterConstants.MINIMUM_ACHIEVABLE_POSITION.getRadians(),
				MAXIMUM_POSITION.getRadians(),
				false,
				Rotation2d.fromDegrees(0).getRadians()
			),
			LifterConstants.GEAR_RATIO
		);

		TalonFXMotor lifter = new TalonFXMotor(logPath, IDs.TalonFXIDs.LIFTER, new SysIdRoutine.Config(), simulation);
		lifter.applyConfiguration(generateMotorConfiguration());
		lifter.setBrake(SET_BRAKE);

		Phoenix6AngleSignal positionSignal = Phoenix6SignalBuilder
			.build(lifter.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);

		return new Lifter(logPath, lifter, positionSignal);
	}

}
