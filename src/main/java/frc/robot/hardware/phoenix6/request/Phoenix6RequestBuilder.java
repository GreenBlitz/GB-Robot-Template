package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.controller.BangBangController;
import frc.utils.angle2d.Angle2d;

import java.util.function.Supplier;

public class Phoenix6RequestBuilder {

	public static Phoenix6VelocityPositionRequest buildVelocityPositionRequest(
		PositionVoltage positionVoltage,
		double defaultArbitraryFeedForward,
		boolean enableFOC
	) {
		return new Phoenix6VelocityPositionRequest(
			Angle2d.fromRotations(positionVoltage.Position),
			Angle2d.fromRotations(positionVoltage.Velocity),
			positionVoltage.withEnableFOC(enableFOC),
			setPoint -> positionVoltage.withVelocity(setPoint.getRotations()),
			setPoint -> positionVoltage.withPosition(setPoint.getRotations()),
			positionVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6FeedForwardRequest buildVelocityVoltageRequest(
		VelocityVoltage velocityVoltage,
		double defaultArbitraryFeedForward,
		boolean enableFOC
	) {
		return new Phoenix6FeedForwardRequest(
			Angle2d.fromRotations(velocityVoltage.Velocity),
			velocityVoltage.withEnableFOC(enableFOC),
			setPoint -> velocityVoltage.withVelocity(setPoint.getRotations()),
			velocityVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6FeedForwardRequest buildVelocityTorqueCurrentFOCRequest(
		VelocityTorqueCurrentFOC velocityTorqueCurrentFOC,
		double defaultArbitraryFeedForward
	) {
		return new Phoenix6FeedForwardRequest(
			Angle2d.fromRotations(velocityTorqueCurrentFOC.Velocity),
			velocityTorqueCurrentFOC,
			setPoint -> velocityTorqueCurrentFOC.withVelocity(setPoint.getRotations()),
			velocityTorqueCurrentFOC::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6MotionMagicRequest buildMotionMagicRequest(
		MotionMagicVoltage motionMagicVoltage,
		double defaultArbitraryFeedForward,
		boolean enableFOC
	) {
		return new Phoenix6MotionMagicRequest(
			Angle2d.fromRotations(motionMagicVoltage.Position),
			motionMagicVoltage.withEnableFOC(enableFOC),
			setPoint -> motionMagicVoltage.withPosition(setPoint.getRotations()),
			motionMagicVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6DynamicMotionMagicRequest buildDynamicMotionMagicRequest(
		DynamicMotionMagicVoltage dynamicMotionMagicVoltage,
		double defaultArbitraryFeedForward,
		boolean enableFOC
	) {
		return new Phoenix6DynamicMotionMagicRequest(
			Angle2d.fromRotations(dynamicMotionMagicVoltage.Position),
			dynamicMotionMagicVoltage.withEnableFOC(enableFOC),
			setPoint -> dynamicMotionMagicVoltage.withPosition(setPoint.getRotations()),
			dynamicMotionMagicVoltage::withFeedForward,
			maxVelocity -> dynamicMotionMagicVoltage.withVelocity(maxVelocity.getRotations()),
			maxAcceleration -> dynamicMotionMagicVoltage.withAcceleration(maxAcceleration.getRotations()),
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6Request<Double> buildVoltageOutRequest(VoltageOut voltageOut, boolean enableFOC) {
		return new Phoenix6Request<>(voltageOut.Output, voltageOut.withEnableFOC(enableFOC), voltageOut::withOutput);
	}

	public static Phoenix6Request<Double> buildTorqueCurrentFOCRequest(TorqueCurrentFOC torqueCurrentFOC) {
		return new Phoenix6Request<>(torqueCurrentFOC.Output, torqueCurrentFOC, torqueCurrentFOC::withOutput);
	}

	public static Phoenix6Request<Angle2d> buildBangBangRequest(
		Supplier<Angle2d> currentVelocity,
		double maxForwardPower,
		double maxBackwardPower,
		boolean enableFOC
	) {
		BangBangController bangBangController = new BangBangController();
		DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(enableFOC);
		return new Phoenix6Request<>(
			Angle2d.kZero,
			dutyCycleOut,
			(Angle2d targetVelocity) -> dutyCycleOut.withOutput(
				bangBangController.calculate(currentVelocity.get().getRotations(), targetVelocity.getRotations()) == 0
					? maxBackwardPower
					: maxForwardPower
			)
		);
	}

}
