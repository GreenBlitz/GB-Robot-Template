package frc.utils;

/**
 * A functional interface that is like the Function interface and the BiFunction but for 3 values as input
 */
public interface TriFunction<In1, In2, In3, Out> {

	Out apply(In1 var1, In2 var2, In3 var3);

}
