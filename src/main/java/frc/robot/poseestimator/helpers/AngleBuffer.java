package frc.robot.poseestimator.helpers;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.helpers.RingBuffer.RingBuffer;
import frc.utils.AngleUtils;
import frc.utils.Filter;

public class AngleBuffer {

	private final RingBuffer<Rotation2d> angleBuffer;

	public AngleBuffer(int angleBufferSize) {
		this.angleBuffer = new RingBuffer<>(angleBufferSize);
	}

	public int size() {
		return angleBuffer.size();
	}

	public void addAngle(Rotation2d angle) {
		angleBuffer.insert(angle);
	}

	public void clear() {
		angleBuffer.clear();
	}

	public int filledAngleSlots() {
		return angleBuffer.filledSlots();
	}

	public Optional<Rotation2d> average() {
		return averageWithFilter(new Filter<>(angle -> true));
	}

	public Optional<Rotation2d> averageWithFilter(Filter<Rotation2d> filter) {
		double sinTotal = 0;
		double cosTotal = 0;

		for (Rotation2d angle : angleBuffer) {
			if (filter.apply(angle)) {
				sinTotal += angle.getSin();
				cosTotal += angle.getCos();
			}
		}

		double sinsDividedByCoses = sinTotal / cosTotal;
		if (Double.isNaN(sinsDividedByCoses)) {
			return Optional.empty();
		}
		return Optional.of(AngleUtils.wrappingAbs(Rotation2d.fromRadians(Math.atan(sinsDividedByCoses))));
	}

}
