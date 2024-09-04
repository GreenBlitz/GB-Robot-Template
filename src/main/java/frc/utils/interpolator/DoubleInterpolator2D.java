package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.Translation2dUtils;

import java.util.*;


public class DoubleInterpolator2D {

	private final HashMap<Translation2d, Double> dataMap;

	public DoubleInterpolator2D() {
		this.dataMap = new HashMap<>();
	}

	public void put(Translation2d point, double value) {
		dataMap.put(point, value);
	}

	public void empty(){
		dataMap.clear();
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
		return InterpolationUtils.interpolate(
			closestTranslations.getFirst(),
			dataMap.get(closestTranslations.getFirst()),
			closestTranslations.getSecond(),
			dataMap.get(closestTranslations.getSecond()),
			targetPoint
		);
	}

	public static Pair<Translation2d, Translation2d> getTranslationPair(Translation2d target,Translation2d[] knownPoints) {
		if (knownPoints.length == 1) {
			Translation2d knownPoint = knownPoints[0];
			return new Pair<>(knownPoint, knownPoint);
		}
		if (knownPoints.length == 2) {
			if (new Translation2d().getDistance(knownPoints[0]) < new Translation2d().getDistance(knownPoints[1])) {
				return new Pair<>(knownPoints[0], knownPoints[1]);
			} else {
				return new Pair<>(knownPoints[1], knownPoints[0]);
			}
		}
		return new Pair<>(Translation2dUtils.findClosestPointBefore(target, knownPoints), Translation2dUtils.findClosestPointAfter(target, knownPoints));
	}

}
