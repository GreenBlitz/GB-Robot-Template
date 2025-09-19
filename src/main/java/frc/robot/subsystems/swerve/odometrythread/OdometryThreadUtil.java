package frc.robot.subsystems.swerve.odometrythread;


import java.util.function.Function;

public class OdometryThreadUtil {

	public static <T> T[] addToFullArray(T toAdd, T[] array, Function<Integer, T[]> arrayConstructor) {
		T[] newArray = arrayConstructor.apply(array.length + 1);
		System.arraycopy(array, 0, newArray, 0, array.length);
		newArray[array.length] = toAdd;
		return newArray;
	}

}
