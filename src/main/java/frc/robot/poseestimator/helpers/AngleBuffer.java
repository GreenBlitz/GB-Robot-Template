package frc.robot.poseestimator.helpers;

import java.util.Optional;
import java.util.function.Predicate;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.MathConstants;
import frc.robot.poseestimator.helpers.RingBuffer.RingBuffer;
import frc.utils.AngleUtils;
import frc.utils.Filter;

public class AngleBuffer {

	private final RingBuffer<Double> angleRadBuffer;

	public AngleBuffer(int angleBufferSize) {
		this.angleRadBuffer = new RingBuffer<>(angleBufferSize);
	}

	public int size() {
		return angleRadBuffer.size();
	}

	public void addAngle(Rotation2d angle) {
		angleRadBuffer.insert(AngleUtils.wrappingAbs(angle).getRadians());
	}

	public void clear() {
		angleRadBuffer.clear();
	}

	public int filledAngleSlots() {
		return angleRadBuffer.filledSlots();
	}

	public Optional<Rotation2d> average() {
		return averageWithFilter(new Filter<>(angle -> true));
	}

	public Optional<Rotation2d> averageWithFilter(Filter<Rotation2d> filter) {
		Predicate<Double> predicateRad = (angleRad) -> filter.apply(Rotation2d.fromRadians(angleRad));
		double totalUnderPi = 0, totalAbovePi = 0;
		double underPiCount = 0, abovePiCount = 0;
		for (double data : angleRadBuffer) {
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
