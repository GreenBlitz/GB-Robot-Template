package frc.robot.vision.cameras.photon;

import frc.utils.math.StandardDeviations2D;

import java.util.function.Function;

public final class PhotonConstants {

	public static final Function<Double, StandardDeviations2D> DEFAULT_STDDEV_FUNCTION_BY_AMBIGUITY = (ambiguity) -> {
		return new StandardDeviations2D(ambiguity, ambiguity, ambiguity);
	};

}
