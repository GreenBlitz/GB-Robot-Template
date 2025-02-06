package frc.robot.subsystems.arm.factory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.empties.EmptyAngleEncoder;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.utils.math.AngleUnit;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class KrakenX60ArmBuilder {

	private static final boolean ENABLE_FOC = true;
	private static final boolean IS_INVERTED = true;
	private static final Rotation2d STARTING_POSITION = Rotation2d.fromDegrees(17);
	private static final int NUMBER_OF_MOTORS = 1;
	private static final double GEAR_RATIO = 450.0 / 7.0;
	public static final double kG = 0;

	protected static Arm build(String logPath) {
		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder.build(
			new DynamicMotionMagicVoltage(
				0,
				ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND.getRotations(),
				ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED.getRotations(),
				0
			),
			0,
			ENABLE_FOC
		);
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), ENABLE_FOC);

		TalonFXMotor motor = new TalonFXMotor(logPath, IDs.TalonFXIDs.ARM, buildSysidConfig(), buildArmSimulation());
		motor.applyConfiguration(buildTalonFXConfiguration());

		Phoenix6AngleSignal motorPositionSignal = Phoenix6SignalBuilder
			.build(motor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.build(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		IAngleEncoder encoder = getEncoder(logPath);

		InputSignal<Rotation2d> encoderPositionSignal = generateEncoderPositionSignal(encoder);

		return new Arm(logPath, motor, positionRequest, voltageRequest, motorPositionSignal, voltageSignal, encoder, encoderPositionSignal);
	}


	public static SysIdRoutine.Config buildSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(0.5).per(Second),
			Volts.of(2),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration buildTalonFXConfiguration() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.MotorOutput.Inverted = IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		switch (Robot.ROBOT_TYPE) {
			case REAL -> {
				config.Slot0.kP = 0;
				config.Slot0.kI = 0;
				config.Slot0.kD = 0;
				config.Slot0.kS = 0.2401653;
				config.Slot0.kG = 0.255;
			}
			case SIMULATION -> {
				config.Slot0.kP = 70;
				config.Slot0.kI = 0;
				config.Slot0.kD = 0;
				config.Slot0.kS = 0;
				config.Slot0.kG = 0;
			}
		}

		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		config.CurrentLimits.SupplyCurrentLimit = 30;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 40;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.FORWARD_SOFTWARE_LIMIT.getRotations();
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.REVERSED_SOFTWARE_LIMIT.getRotations();
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

		config.Feedback.RotorToSensorRatio = GEAR_RATIO;
		config.Feedback.SensorToMechanismRatio = 1;

		config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		config.Feedback.FeedbackRemoteSensorID = IDs.CANCodersIDs.ARM.id();

		return config;
	}

	private static SingleJointedArmSimulation buildArmSimulation() {
		return new SingleJointedArmSimulation(
			new SingleJointedArmSim(
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS),
					SingleJointedArmSim.estimateMOI(ArmConstants.LENGTH_METERS, ArmConstants.MASS_KG),
					GEAR_RATIO
				),
				DCMotor.getKrakenX60(NUMBER_OF_MOTORS),
				GEAR_RATIO,
				ArmConstants.LENGTH_METERS,
				ArmConstants.REVERSED_SOFTWARE_LIMIT.getRadians(),
				ArmConstants.FORWARD_SOFTWARE_LIMIT.getRadians(),
				false,
				STARTING_POSITION.getRadians()
			),
			GEAR_RATIO
		);
	}

	private static IAngleEncoder getEncoder(String logPath) {
		if (Robot.ROBOT_TYPE.isReal()) {
			CANCoderEncoder encoder = new CANCoderEncoder(
					logPath + "/Encoder",
					new CANcoder(IDs.CANCodersIDs.ARM.id(), IDs.CANCodersIDs.ARM.busChain().getChainName())
			);
			CANcoderConfiguration configuration = new CANcoderConfiguration();
			configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
			return encoder;
		}
		return new EmptyAngleEncoder(logPath + "/Encoder");
	}

	private static InputSignal<Rotation2d> generateEncoderPositionSignal(IAngleEncoder encoder) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL ->
				Phoenix6SignalBuilder.build(
					((CANCoderEncoder) encoder).getDevice().getPosition(),
					RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
					AngleUnit.ROTATIONS
				);
			case SIMULATION -> new SuppliedAngleSignal("position", () -> 0.0, AngleUnit.ROTATIONS);
		};
	}

}
