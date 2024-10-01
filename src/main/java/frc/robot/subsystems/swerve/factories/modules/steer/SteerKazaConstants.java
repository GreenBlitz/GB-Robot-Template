package frc.robot.subsystems.swerve.factories.modules.steer;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.MathConstants;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxDoubleRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.swerve.modules.stuffs.SteerStuff;
import frc.utils.AngleUnit;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class SteerKazaConstants {

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(0.5).per(Seconds.of(1)),
			Volts.of(1),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	protected static SteerStuff generateSteerStuff(String logPath, SparkMaxDeviceID deviceID, boolean inverted) {
		SparkMaxAngleRequest positionRequest = new SparkMaxAngleRequest(
			new Rotation2d(0),
			SparkMaxAngleRequest.SparkAngleRequestType.POSITION,
			0
		);
		SparkMaxDoubleRequest voltageRequest = new SparkMaxDoubleRequest(0, SparkMaxDoubleRequest.SparkDoubleRequestType.VOLTAGE, 0);

		SparkMaxWrapper motor = new SparkMaxWrapper(deviceID);
		motor.setInverted(inverted);

		BrushlessSparkMAXMotor steer = new BrushlessSparkMAXMotor(logPath, motor, generateSysidConfig());

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", () -> motor.getAppliedOutput() * motor.getBusVoltage());
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("current", motor::getOutputCurrent);
		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", () -> motor.getEncoder().getPosition(), AngleUnit.ROTATIONS);
		//@formatter:off
		SuppliedAngleSignal velocitySignal = new SuppliedAngleSignal("velocity", () -> motor.getEncoder().getVelocity() * MathConstants.MINUTES_TO_SECONDS_CONVERSION_FACTOR, AngleUnit.ROTATIONS);
		//@formatter:on

		return new SteerStuff(steer, positionRequest, voltageRequest, positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

}
