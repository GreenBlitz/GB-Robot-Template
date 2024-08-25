package frc.utils;

import java.util.function.Consumer;

public class LoopingUtils {

	public static void loop(Consumer<Integer> loop, int iterations) {
		for (int index = 0; index < iterations; index++) {
			loop.accept(index);
		}
	}

}
