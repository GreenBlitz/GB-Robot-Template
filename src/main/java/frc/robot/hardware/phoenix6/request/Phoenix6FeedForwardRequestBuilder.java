package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.BooleanSupplier;

public class Phoenix6FeedForwardRequestBuilder extends Phoenix6RequestBuilder {

	public static Phoenix6FeedForwardRequest build(
		PositionVoltage positionVoltage,
		double defaultArbitraryFeedForward,
		BooleanSupplier applyFeedForward
	) {
		positionVoltage.withFeedForward(defaultArbitraryFeedForward);
		return new Phoenix6FeedForwardRequest(
			Rotation2d.fromRotations(positionVoltage.Position),
			positionVoltage,
			setPoint -> positionVoltage.withPosition(setPoint.getRotations()),
			positionVoltage::withFeedForward,
			applyFeedForward,
			defaultArbitraryFeedForward
		);
	}

}
