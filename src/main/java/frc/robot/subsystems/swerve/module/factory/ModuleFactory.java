package frc.robot.subsystems.swerve.module.factory;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleRequests;
import frc.robot.subsystems.swerve.module.ModuleSignals;
import frc.utils.AngleUnit;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class ModuleFactory {

	public static final boolean enableFOC = true;

	public static boolean ARE_MOTORS_CTRE = true;

	public static final double DEFAULT_ARBITRARY_FEED_FORWARD = 0;


	public static Module build(RealModuleConstants constants) {
		String finalLogPath = ModuleConstants.LOG_PATH_PREFIX + constants.logPath();

		SysIdCalibrator.SysIdConfigInfo configInfo = new SysIdCalibrator.SysIdConfigInfo(
			new SysIdRoutine.Config(),
			ARE_MOTORS_CTRE
		);

		TalonFXMotor drive = generateDrive(finalLogPath, constants);
		TalonFXMotor steer = generateSteer(finalLogPath, constants);
		CANCoderEncoder encoder = generateEncoder(finalLogPath, constants);

		ModuleRequests requests = generateRequests();
		ModuleSignals signals = generateSignals(drive, steer, encoder);

		return new Module(finalLogPath, drive, steer, encoder, requests, signals, configInfo, constants);
	}

	private static TalonFXMotor generateDrive(String logPath, RealModuleConstants constants) {
		return new TalonFXMotor(logPath + "/Drive", new Phoenix6DeviceID(constants.driveMotorId(), BusChain.ROBORIO), new SysIdRoutine.Config());
	}

	private static TalonFXMotor generateSteer(String logPath, RealModuleConstants constants) {
		return new TalonFXMotor(logPath + "/Steer", new Phoenix6DeviceID(constants.steerMotorId(), BusChain.ROBORIO), new SysIdRoutine.Config());
	}

	private static CANCoderEncoder generateEncoder(String logPath, RealModuleConstants constants) {
		return new CANCoderEncoder(logPath + "/Encoder", new CANcoder(constants.encoderId(), BusChain.ROBORIO.getChainName()));
	}

	private static ModuleRequests generateRequests() {
		IRequest<Rotation2d> driveVelocityRequest = Phoenix6RequestBuilder
			.build(new VelocityVoltage(0), DEFAULT_ARBITRARY_FEED_FORWARD, enableFOC);
		IRequest<Double> driveVoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), enableFOC);
		IRequest<Rotation2d> steerAngleRequest = Phoenix6RequestBuilder
			.build(new PositionVoltage(0), DEFAULT_ARBITRARY_FEED_FORWARD, enableFOC);

		return new ModuleRequests(driveVelocityRequest, driveVoltageRequest, steerAngleRequest);
	}

	private static ModuleSignals generateSignals(TalonFXMotor drive, TalonFXMotor steer, CANCoderEncoder encoder) {
		InputSignal<Rotation2d> driveVelocitySignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		InputSignal<Double> driveVoltageSignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
		InputSignal<Double> driveCurrentSignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getMotorStallCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
		InputSignal<Rotation2d> steerAngleSignal = Phoenix6SignalBuilder
			.build(steer.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		InputSignal<Double> steerVoltageSignal = Phoenix6SignalBuilder
			.build(steer.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
		InputSignal<Rotation2d> encoderAngleSignal = Phoenix6SignalBuilder
			.build(encoder.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);


		return new ModuleSignals(
			driveVelocitySignal,
			driveVoltageSignal,
			driveCurrentSignal,
			steerAngleSignal,
			steerVoltageSignal,
			encoderAngleSignal
		);
	}

}
