package frc.robot.poseestimator.helpers;

import java.util.Optional;
import java.util.function.Predicate;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.MathConstants;

public class AngleBuffer {

	private final RingBuffer<Double> angleRadBuffer;

	public AngleBuffer(int angleBufferSize) {
		this.angleRadBuffer = new RingBuffer<>(angleBufferSize);
	}

	public int size() {
		return angleRadBuffer.size();
	}

	public void addAngle(Rotation2d angle) {
		angleRadBuffer.insert((angle.getRadians() + MathConstants.FULL_CIRCLE.getRadians()) % MathConstants.FULL_CIRCLE.getRadians());
	}

	public void clear() {
		angleRadBuffer.clear();
	}

	public int filledAngleSlots() {
		return angleRadBuffer.filledSlots();
	}

	public Optional<Rotation2d> average() {
		return averageWithPredicate((angle) -> true);
	}

	public Optional<Rotation2d> averageWithPredicate(Predicate<Rotation2d> predicate) {
		Predicate<Double> predicateRad = (angleRad) -> predicate.test(Rotation2d.fromRadians(angleRad));
		double totalUnderPi = 0, totalAbovePi = 0;
		double underPiCount = 0, abovePiCount = 0;
		for (Double data : angleRadBuffer) {
			if (predicateRad.test(data)) {
				if (data < MathConstants.HALF_CIRCLE.getRadians()) {
					totalUnderPi += data;
					underPiCount++;
				} else {
					totalAbovePi += data;
					abovePiCount++;
				}
			}
		}
		double angleCount = underPiCount + abovePiCount;
		double underPiRatio = underPiCount / angleCount;
		double abovePiRatio = abovePiCount / angleCount;
		totalAbovePi /= abovePiCount;
		totalUnderPi /= underPiCount;
		if (Double.isNaN(totalUnderPi)) {
			totalUnderPi = 0;
		}
		if (Double.isNaN(totalAbovePi)) {
			totalAbovePi = 0;
		}
		if (totalUnderPi == 0 && totalAbovePi == 0) {
			return Optional.empty();
		}
		totalAbovePi -= MathConstants.HALF_CIRCLE.getRadians();
		return Optional.of(
			Rotation2d.fromRadians(
				Math.atan2(
					(Math.sin(totalUnderPi) * underPiRatio + Math.sin(totalAbovePi) * abovePiRatio),
					(Math.cos(totalUnderPi) * underPiRatio + Math.cos(totalAbovePi) * abovePiRatio)
				)
			)
		);
	}

}
