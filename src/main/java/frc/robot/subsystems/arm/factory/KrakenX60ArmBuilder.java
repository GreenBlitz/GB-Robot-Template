package frc.robot.subsystems.arm.factory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.constants.MathConstants;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotType;
import frc.robot.hardware.empties.EmptyAngleEncoder;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
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

	static final boolean IS_FOC = true;
	public static final Rotation2d STARTING_POSITION = Rotation2d.fromDegrees(17);

	protected static Arm build(String logPath) {
		Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0).withEnableFOC(IS_FOC));
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(IS_FOC));

		TalonFXMotor motor = new TalonFXMotor(logPath, IDs.TalonFXIDs.ARM_DEVICE_ID, buildSysidConfig(), buildArmSimulation());
		motor.applyConfiguration(buildTalonFXConfiguration(IDs.CANCoderIDs.ARM_CAN_CODER));

		IAngleEncoder encoder = getEncoder(logPath);

		Phoenix6AngleSignal armPositionSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		InputSignal<Rotation2d> encoderPositionSignal = getEncoderPositionSignal(encoder);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		return new Arm(logPath, motor, encoder, positionRequest, voltageRequest, armPositionSignal, encoderPositionSignal, voltageSignal);
	}


	public static SysIdRoutine.Config buildSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(0.5).per(Second),
			Volts.of(2),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration buildTalonFXConfiguration(Phoenix6DeviceID encoderDeviceId) {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Slot0.kP = Robot.ROBOT_TYPE.isSimulation() ? 0.8 : 0.9;
		config.Slot0.kI = Robot.ROBOT_TYPE == RobotType.SIMULATION ? 0.8 : 0.9;
		config.Slot0.kD = Robot.ROBOT_TYPE == RobotType.SIMULATION ? 0.8 : 0.9;
		config.Slot0.kS = Robot.ROBOT_TYPE == RobotType.SIMULATION ? 0 : 0.1;
		config.Slot0.kG = Robot.ROBOT_TYPE == RobotType.SIMULATION ? 0 : 0.1;
		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		config.CurrentLimits.SupplyCurrentLimit = 30;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 40;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotation2d.fromDegrees(ArmConstants.FORWARD_SOFTWARE_LIMIT).getRotations();
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Rotation2d.fromDegrees(ArmConstants.REVERSED_SOFTWARE_LIMIT).getRotations();
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

		config.Feedback.RotorToSensorRatio = ArmConstants.GEAR_RATIO;
		config.Feedback.SensorToMechanismRatio = 1;

		config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		config.Feedback.FeedbackRemoteSensorID = encoderDeviceId.id();

		return config;
	}

	private static SingleJointedArmSimulation buildArmSimulation() {
		return new SingleJointedArmSimulation(
			new SingleJointedArmSim(
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60Foc(ArmConstants.NUMBER_OF_MOTORS),
					SingleJointedArmSim.estimateMOI(ArmConstants.LENGTH_IN_METERS, ArmConstants.MASS_IN_KG),
					ArmConstants.GEAR_RATIO
				),
				DCMotor.getKrakenX60(ArmConstants.NUMBER_OF_MOTORS),
				ArmConstants.GEAR_RATIO,
				ArmConstants.LENGTH_IN_METERS,
				MathConstants.QUARTER_CIRCLE.unaryMinus().getRadians(),
				MathConstants.QUARTER_CIRCLE.getRadians(),
				false,
				STARTING_POSITION.getRadians()
			),
			ArmConstants.GEAR_RATIO
		);
	}

	private static IAngleEncoder getEncoder(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL ->
				new CANCoderEncoder(logPath + "/Encoder", new CANcoder(IDs.CANCoderIDs.ARM_CAN_CODER.id(), BusChain.ROBORIO.getChainName()));
			case SIMULATION -> new EmptyAngleEncoder(logPath + "/Encoder");
		};
	}

	private static InputSignal<Rotation2d> getEncoderPositionSignal(IAngleEncoder encoder) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL ->
				Phoenix6SignalBuilder.generatePhoenix6Signal(
					((CANCoderEncoder) encoder).getDevice().getPosition(),
					RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
					AngleUnit.ROTATIONS
				);
			case SIMULATION -> new SuppliedAngleSignal("encoderPositionSignal", () -> 0.0, AngleUnit.ROTATIONS);
		};
	}

}
