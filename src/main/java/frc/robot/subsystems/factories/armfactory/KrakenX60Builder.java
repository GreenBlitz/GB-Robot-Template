package frc.robot.subsystems.factories.armfactory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.constants.MathConstants;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.arm.Arm;
import frc.utils.math.AngleUnit;

public class KrakenX60Builder {

	static final boolean IS_FOC = true;
	static final double KP = 1;
	static final double KI = 0.8;
	static final double KD = 0.8;
	static final double KS = 0;
	static final double KG = 0;
	static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
	static final int SUPPLY_CURRENT_LIMIT = 30;
	static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
	static final int STATOR_CURRENT_LIMIT = 40;
	static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
	static final double REVERSED_SOFTWARE_LIMIT_THRESHOLD = Rotation2d.fromDegrees(17).getRotations();
	static final boolean REVERSED_SOFTWARE_LIMIT_THRESHOLD_ENABLE = true;
	static final double FORWARD_SOFTWARE_LIMIT_THRESHOLD = Rotation2d.fromDegrees(50).getRotations();
	static final boolean FORWARD_SOFTWARE_LIMIT_THRESHOLD_ENABLE = true;

	protected static Arm build(String logPath) {
		Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder
			.build(new PositionVoltage(ArmConstants.STARTING_POSITION.getDegrees()).withEnableFOC(IS_FOC));
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(ArmConstants.STARTING_VOLTAGE).withEnableFOC(IS_FOC));

		SingleJointedArmSim armSim = new SingleJointedArmSim(
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
			ArmConstants.STARTING_POSITION.getRadians()
		);

		SingleJointedArmSimulation armSimulation = new SingleJointedArmSimulation(armSim, ArmConstants.GEAR_RATIO);

		TalonFXMotor motor = new TalonFXMotor(logPath, IDs.TalonFXIDs.ARM_DEVICE_ID, buildSysidConfig(), armSimulation);
		motor.applyConfiguration(buildTalonFXConfiguration());

		Phoenix6AngleSignal positionSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		return new Arm(logPath, motor, positionRequest, voltageRequest, positionSignal, voltageSignal);
	}


	public static SysIdRoutine.Config buildSysidConfig() {
		return new SysIdRoutine.Config(null, null, null, state -> SignalLogger.writeString("state", state.toString()));
	}

	private static TalonFXConfiguration buildTalonFXConfiguration() {
		return new TalonFXConfiguration()
			.withSlot0(new Slot0Configs().withKP(KP).withKI(KI).withKD(KD).withKS(KS).withKS(KS).withGravityType(GRAVITY_TYPE))
			.withCurrentLimits(
				new CurrentLimitsConfigs().withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
					.withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
					.withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
					.withStatorCurrentLimitEnable(STATOR_CURRENT_LIMIT_ENABLE)
			)
			.withSoftwareLimitSwitch(
				new SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(FORWARD_SOFTWARE_LIMIT_THRESHOLD)
					.withForwardSoftLimitEnable(FORWARD_SOFTWARE_LIMIT_THRESHOLD_ENABLE)
					.withReverseSoftLimitThreshold(REVERSED_SOFTWARE_LIMIT_THRESHOLD)
					.withReverseSoftLimitEnable(REVERSED_SOFTWARE_LIMIT_THRESHOLD_ENABLE)
			)
			.withFeedback(
				new FeedbackConfigs().withRotorToSensorRatio(ArmConstants.GEAR_RATIO)
					.withSensorToMechanismRatio(ArmConstants.SENSOR_TO_MECHANISM_RATIO)
					.withFusedCANcoder(new CoreCANcoder(IDs.CANCodersIDs.ARM_CAN_CODER.getDeviceID()))
			);
	}

}
