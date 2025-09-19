package frc.robot.subsystems.swerve.odometrythread;

import com.ctre.phoenix6.StatusSignal;

import java.util.Arrays;
import java.util.function.Function;

public class OdometryThreadUtil {

	public static double calculateAverageLatency(StatusSignal<?>[] signals) {
		if (signals.length == 0) {
			return 0;
		}
		double latencySum = Arrays.stream(signals).mapToDouble(signal -> signal.getTimestamp().getLatency()).sum();
		return latencySum / signals.length;
	}

	public static <T> T[] addToFullArray(T toAdd, T[] array, Function<Integer, T[]> arrayConstructor) {
		T[] newArray = arrayConstructor.apply(array.length + 1);
		System.arraycopy(array, 0, newArray, 0, array.length);
		newArray[array.length] = toAdd;
		return newArray;
	}

}
