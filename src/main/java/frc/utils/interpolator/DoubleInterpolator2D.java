package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Arrays;
import java.util.HashMap;

import static frc.utils.Translation2dUtils.getNegativeClosest;
import static frc.utils.Translation2dUtils.getPositiveClosest;

public class DoubleInterpolator2D {

	private final HashMap<Translation2d, Double> dataMap;

	public DoubleInterpolator2D() {
		this.dataMap = new HashMap<>();
	}

	public void put(Translation2d point, double value) {
		dataMap.put(point, value);
	}

	public double get(Translation2d targetPoint) {
		if (dataMap.isEmpty()) {
			return 0;
		}
		if (dataMap.containsKey(targetPoint)) {
			return dataMap.get(targetPoint);
		}
		Pair<Translation2d, Translation2d> closestTranslations = getTranslationPair(targetPoint, dataMap.keySet().toArray(new Translation2d[0]));
		System.out.println(closestTranslations.getFirst());
		System.out.println(closestTranslations.getSecond());

		return InterpolationUtils.linearInterpolation2d(
			closestTranslations.getFirst(),
			dataMap.get(closestTranslations.getFirst()),
			closestTranslations.getSecond(),
			dataMap.get(closestTranslations.getSecond()),
			targetPoint
		);
	}

	private Pair<Translation2d, Translation2d> getTranslationPair(Translation2d target, Translation2d... knownPoints) {
		if (knownPoints.length == 1) {
			Translation2d knownPoint = (Translation2d) Arrays.stream(knownPoints).toArray()[0];
			return new Pair<>(knownPoint, knownPoint);
		}
		if (knownPoints.length == 2) {
			Translation2d[] knownPointsArray = (Translation2d[]) Arrays.stream(knownPoints).toArray();
			if (new Translation2d().getDistance(knownPointsArray[0]) < new Translation2d().getDistance(knownPointsArray[1])) {
				return new Pair<>(knownPointsArray[0], knownPointsArray[1]);
			} else {
				return new Pair<>(knownPointsArray[1], knownPointsArray[0]);
			}
		}
		return new Pair<>(getNegativeClosest(target, knownPoints), getPositiveClosest(target, knownPoints));
	}

}
