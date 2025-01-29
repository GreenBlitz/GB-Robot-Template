package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.geometry.Rotation2d;

public class Phoenix6RequestBuilder {

	public static Phoenix6Request<Rotation2d> build(PositionVoltage positionVoltage) {
		return new Phoenix6Request<>(
			Rotation2d.fromRotations(positionVoltage.Position),
			positionVoltage,
			setPoint -> positionVoltage.withPosition(setPoint.getRotations())
		);
	}

	public static Phoenix6Request<Rotation2d> build(MotionMagicDutyCycle MotionMagicDutyCycle) {
		return new Phoenix6Request<>(
			Rotation2d.fromRotations(MotionMagicDutyCycle.Position),
			MotionMagicDutyCycle,
			setPoint -> MotionMagicDutyCycle.withPosition(setPoint.getRotations())
		);
	}

	public static Phoenix6Request<Rotation2d> build(DynamicMotionMagicDutyCycle dynamicMotionMagicDutyCycle) {
		return new Phoenix6Request<>(
			Rotation2d.fromRotations(dynamicMotionMagicDutyCycle.Position),
			dynamicMotionMagicDutyCycle,
			setPoint -> dynamicMotionMagicDutyCycle.withPosition(setPoint.getRotations())
		);
	}

	public static Phoenix6Request<Rotation2d> build(VelocityVoltage velocityVoltage) {
		return new Phoenix6Request<>(
			Rotation2d.fromRotations(velocityVoltage.Velocity),
			velocityVoltage,
			setPoint -> velocityVoltage.withVelocity(setPoint.getRotations())
		);
	}

	public static Phoenix6Request<Double> build(VoltageOut voltageOut) {
		return new Phoenix6Request<>(voltageOut.Output, voltageOut, voltageOut::withOutput);
	}

	public static Phoenix6Request<Rotation2d> build(MotionMagicVoltage motionMagicVoltage) {
		return new Phoenix6Request<>(
			Rotation2d.fromRotations(motionMagicVoltage.Position),
			motionMagicVoltage,
			setPoint -> motionMagicVoltage.withPosition(setPoint.getRotations())
		);
	}

	public static Phoenix6Request<Rotation2d> build(DynamicMotionMagicVoltage dynamicMotionMagicVoltage) {
		return new Phoenix6Request<>(
			Rotation2d.fromRotations(dynamicMotionMagicVoltage.Position),
			dynamicMotionMagicVoltage,
			setPoint -> dynamicMotionMagicVoltage.withPosition(setPoint.getRotations())
		);
	}

	public static Phoenix6Request<Double> build(TorqueCurrentFOC torqueCurrentFOC) {
		return new Phoenix6Request<>(torqueCurrentFOC.Output, torqueCurrentFOC, torqueCurrentFOC::withOutput);
	}

}
