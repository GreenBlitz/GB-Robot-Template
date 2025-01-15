package frc.robot.poseestimator.helpers;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Predicate;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.MathConstants;

public class SlidingWindowAngleAccumulator {

	private final ArrayList<Optional<Double>> angleAccumulatorRad;
	private int nextAngleSlot;

	public SlidingWindowAngleAccumulator(int angleAccumulatorSize) {
		this.angleAccumulatorRad = new ArrayList<>(angleAccumulatorSize);
		for (int i = 0; i < angleAccumulatorSize; i++) {
			angleAccumulatorRad.add(Optional.empty());
		}
		this.nextAngleSlot = 0;
	}

	public int size() {
		return angleAccumulatorRad.size();
	}

	public void addAngle(Rotation2d angle) {
		angleAccumulatorRad.set(
			this.nextAngleSlot++,
			Optional.of((angle.getRadians() + MathConstants.FULL_CIRCLE.getRadians()) % MathConstants.FULL_CIRCLE.getRadians())
		);
		if (nextAngleSlot >= angleAccumulatorRad.size()) {
			nextAngleSlot = 0;
		}
	}

	public void clear() {
		for (int i = 0; i < size(); i++) {
			angleAccumulatorRad.add(Optional.empty());
		}
	}

	public int filledAngleSlots() {
		int filledSlots = 0;
		for (Optional<Double> data : angleAccumulatorRad) {
			if (data.isPresent()) {
				filledSlots++;
			}
		}
		return filledSlots;
	}

	public Optional<Rotation2d> average() {
		return averageWithPredicate((angle) -> true);
	}

	public Optional<Rotation2d> averageWithPredicate(Predicate<Rotation2d> predicate) {
		Predicate<Double> predicateRad = (angleRad) -> predicate.test(Rotation2d.fromRadians(angleRad));
		double totalUnderPi = 0, totalAbovePi = 0;
		double underPiCount = 0, abovePiCount = 0;
		for (Optional<Double> data : angleAccumulatorRad) {
			if (data.isPresent() && predicateRad.test(data.get())) {
				if (data.get() < MathConstants.HALF_CIRCLE.getRadians()) {
					totalUnderPi += data.get();
					underPiCount++;
				} else {
					totalAbovePi += data.get();
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
