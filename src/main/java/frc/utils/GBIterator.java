package frc.utils;

import java.util.function.Consumer;

public class GBIterator {

	public static void loop(Consumer<Integer> iterator, int iterations) {
		for (int index = 0; index < iterations; index++) {
			iterator.accept(index);
		}
	}

}
