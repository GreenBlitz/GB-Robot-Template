package frc.utils.gbmeth;

import edu.wpi.first.math.Num;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.ejml.simple.SimpleMatrix;

import java.util.*;
import java.util.function.Function;
import java.util.stream.StreamSupport;

public class GBVector<S extends Num> implements Cloneable, Iterable<Double>, Vector<S> {

	private GBVector<?> cloneOf;
	private boolean clone;
	private double[] data;
	private double factorOf;
	private Function<Double, Double> appliedFunction;

	private GBVector(GBVector<?> anotherVector) {
		this.cloneOf = anotherVector;
		this.clone = true;
		this.data = null;
		this.factorOf = 1;
		this.appliedFunction = anotherVector.appliedFunction;
	}

	public GBVector(double[] data) {
		this.data = data;
		this.clone = false;
		this.factorOf = 1;
		this.cloneOf = null;
		this.appliedFunction = x -> x;
	}

	public <E extends Num> GBVector(edu.wpi.first.math.Vector<E> vector) {
		this(vector.getData());
	}

	public GBVector(Collection<Double> data) {
		this(data.stream().mapToDouble(x -> x).toArray());
	}

	@Override
	public final Iterator<Double> iterator() {
		return new Iterator<>() {

			private int pointer = 0;

			@Override
			public boolean hasNext() {
				return pointer < data.length;
			}

			@Override
			public Double next() {
				return get(pointer++);
			}

		};
	}

	public final double get(int index) {
		double output;
		if (!clone) {
			output = data[index];
		} else {
			output = cloneOf.get(index);
		}
		return appliedFunction.apply(output * factorOf);
	}

	@Override
	public final double dot(Vector<S> anotherVector) {
		double output = 0;
		for (int i = 0; i < assertSizeGetMinimum(anotherVector); i++) {
			output += anotherVector.get(i) * this.get(i);
		}
		return output;
	}

	public final void set(int index, double value) {
		if (clone) {
			this.data = cloneOf.data.clone();
			this.clone = false;
			this.factorOf *= cloneOf.factorOf;
			this.cloneOf = null; // frees up the garbage collector
		}
		data[index] = value;
	}

	@Override
	public void plus(Vector<S> anotherVector) {
		for (int i = 0; i < assertSizeGetMinimum(anotherVector); i++) {
			this.set(i, this.get(i) + anotherVector.get(i));
		}
	}

	@Override
	public void minus(Vector<S> anotherVector) {
		for (int i = 0; i < assertSizeGetMinimum(anotherVector); i++) {
			this.set(i, this.get(i) - anotherVector.get(i));
		}
	}

	@Override
	public final void divide(double factor) {
		this.factorOf /= factor;
	}

	@Override
	public final void multiple(double factor) {
		this.factorOf *= factor;
	}

	@Override
	public final void invert() {
		this.multiple(-1);
	}

	public final int size() {
		return data.length;
	}

	public final Vector<S> unit() {
		GBVector<S> clonedVector = this.clone();
		clonedVector.divide(norm());
		return clonedVector;
	}

	public final double norm() {
		return Math
			.sqrt(StreamSupport.stream(this.spliterator(), false).map(x -> Math.pow(x, 2)).reduce(Double::sum).orElseGet(() -> Double.NaN));
	}

	private double assertSizeGetMinimum(Vector<?> anotherVector) {
		int theirSize = anotherVector.size();
		int mySize = this.size();
		if (mySize != theirSize) {
			AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.ERROR, "GotBadVectorSizeExpected" + mySize + "Got" + theirSize, () -> true));
		}
		return Math.min(theirSize, mySize);
	}

	public final void injectFunctionOnOutput(Function<Double, Double> injectedFunction) {
		this.appliedFunction = injectedFunction;
	}

	@Override
	public final String toString() {
		StringBuilder output = new StringBuilder();
		for (double x : this) {
			output.append(String.format("%.4f", x));
		}
		return output.toString();
	}

	public final double[] toArray() {
		return this.data.clone();
	}

	public final edu.wpi.first.math.Vector<S> toWPILibVector() {
		return new edu.wpi.first.math.Vector<>(new SimpleMatrix(this.data));
	}

	public final String debugString() {
		return "GBBasisVector{"
			+ ", cloneOf="
			+ cloneOf
			+ ", clone="
			+ clone
			+ ", data="
			+ Arrays.toString(data)
			+ ", factorOf="
			+ factorOf
			+ '}';
	}

	@Override
	public final GBVector<S> clone() {
		GBVector<S> cloned;
		try {
			cloned = (GBVector<S>) super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException(e);
		}
		cloned.appliedFunction = this.appliedFunction;
		this.cloneOf = clone ? this.cloneOf.clone() : null;
		this.clone = true;
		this.data = null;
		this.factorOf = 1;
		return new GBVector<>(this);
	}

	public static <E extends Num> GBVector<E> deepClone(GBVector<E> vector) {
		GBVector<E> cloned = vector.clone();
		cloned.data = vector.data.clone();
		return cloned;
	}

}
