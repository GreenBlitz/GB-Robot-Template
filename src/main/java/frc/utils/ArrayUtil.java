package frc.utils;

import java.util.function.Function;

public class ArrayUtil {

	public static <T, E extends T> T[] addToFullArray(E toAdd, T[] array, Function<Integer, T[]> arrayConstructor) {
		T[] newArray = arrayConstructor.apply(array.length + 1);
		System.arraycopy(array, 0, newArray, 0, array.length);
		newArray[array.length] = toAdd;
		return newArray;
	}

}
