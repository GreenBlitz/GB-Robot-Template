package frc.robot.subsystems.arm.factory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.hardware.phoenix6.Phoenix6Util;
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
import frc.utils.alerts.Alert;
import frc.utils.math.AngleUnit;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class KrakenX60ArmBuilder {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private static final boolean ENABLE_FOC = true;
	private static final boolean IS_INVERTED = true;
	private static final Rotation2d STARTING_POSITION = Rotation2d.fromDegrees(17);
	private static final int NUMBER_OF_MOTORS = 1;
	private static final double GEAR_RATIO = 450.0 / 7.0;
	public static final double kG = 0.33;

	protected static Arm build(String logPath) {
		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder.build(new MotionMagicVoltage(0).withSlot(0), 0, ENABLE_FOC);
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
		return new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), null, state -> SignalLogger.writeString("state", state.toString()));
	}

	private static TalonFXConfiguration buildTalonFXConfiguration() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		switch (Robot.ROBOT_TYPE) {
			case REAL -> {
				// Motion magic
				config.Slot0.kP = 28;
				config.Slot0.kI = 0;
				config.Slot0.kD = 0;
				config.Slot0.kS = 0.0715;
				config.Slot0.kG = kG;
				config.Slot0.kV = 9.0000095367432;
				config.Slot0.kA = 0.5209;

				// PID
				config.Slot1.kP = 80;
				config.Slot1.kI = 0;
				config.Slot1.kD = 0;
				config.Slot1.kS = 0.0715;
				config.Slot1.kG = kG;
			}
			case SIMULATION -> {
				config.Slot0.kP = 70;
				config.Slot0.kI = 0;
				config.Slot0.kD = 0;
				config.Slot0.kS = 0;
				config.Slot0.kG = 0;

				config.Slot1.kP = 70;
				config.Slot1.kI = 0;
				config.Slot1.kD = 0;
				config.Slot1.kS = 0;
				config.Slot1.kG = 0;
			}
		}
		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
		config.Slot2.GravityType = GravityTypeValue.Arm_Cosine;

		config.MotionMagic.MotionMagicAcceleration = ArmConstants.ACCELERATION_ANGLES_PER_SECOND_SQUARED.getRotations();
		config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CRUISE_VELOCITY_ANGLES_PER_SECOND.getRotations();

		config.MotorOutput.Inverted = IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		config.CurrentLimits.SupplyCurrentLimit = 40;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 40;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.FORWARD_SOFTWARE_LIMIT.getRotations();
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.ELEVATOR_OPEN_REVERSED_SOFTWARE_LIMIT.getRotations();
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
				ArmConstants.ELEVATOR_OPEN_REVERSED_SOFTWARE_LIMIT.getRadians(),
				ArmConstants.MAXIMUM_POSITION.getRadians(),
				false,
				STARTING_POSITION.getRadians()
			),
			GEAR_RATIO
		);
	}

	private static IAngleEncoder getEncoder(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> buildRealEncoder(logPath);
			case SIMULATION -> new EmptyAngleEncoder(logPath + "/Encoder");
		};
	}

	private static IAngleEncoder buildRealEncoder(String logPath) {
		CANcoder canCoder = new CANcoder(IDs.CANCodersIDs.ARM.id(), IDs.CANCodersIDs.ARM.busChain().getChainName());
		CANCoderEncoder encoder = new CANCoderEncoder(logPath + "/Encoder", canCoder);
		CANcoderConfiguration configuration = buildEncoderConfig(encoder);
		if (
			!Phoenix6Util.checkStatusCodeWithRetry(() -> encoder.getDevice().getConfigurator().apply(configuration), APPLY_CONFIG_RETRIES).isOK()
		) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}

		return encoder;
	}

	private static CANcoderConfiguration buildEncoderConfig(CANCoderEncoder canCoderEncoder) {
		MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
		canCoderEncoder.getDevice().getConfigurator().refresh(magnetSensorConfigs);

		CANcoderConfiguration configuration = new CANcoderConfiguration();
		configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		configuration.MagnetSensor.MagnetOffset = magnetSensorConfigs.MagnetOffset;
		configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ArmConstants.MAXIMUM_POSITION.getRotations();

		return configuration;
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
