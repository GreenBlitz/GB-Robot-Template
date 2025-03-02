package frc.utils.gbmeth;

import edu.wpi.first.math.Num;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;

import java.util.*;
import java.util.function.Function;
import java.util.stream.StreamSupport;

public final class GBBasisVector<S extends Num> implements Cloneable, Iterable<Double>, Vector<S> {

	private final int[] minorOf;

	private GBBasisVector<?> cloneOf;
	private boolean clone;
	private double[] data;
	private double factorOf;
	private Function<Double, Double> appliedFunction;

	private GBBasisVector(GBBasisVector<?> anotherVector, boolean clone, int[] minorOf) {
		this.minorOf = minorOf;
		this.appliedFunction = anotherVector.appliedFunction;

		this.cloneOf = anotherVector;
		this.clone = clone;
		this.data = null;
		this.factorOf = 1;
	}

	private GBBasisVector(GBBasisVector<?> anotherVector, boolean clone) {
		this(anotherVector, clone, new int[anotherVector.getSize()]);
	}

	public GBBasisVector(double[] data) {
		this.data = data;
		this.clone = false;
		this.factorOf = 1;
		this.cloneOf = null;
		this.minorOf = null;
		this.appliedFunction = x -> x;
	}

	public <E extends Num> GBBasisVector(edu.wpi.first.math.Vector<E> vector) {
		this(vector.getData());
	}

	public GBBasisVector(Collection<Double> data) {
		this(data.stream().mapToDouble(x -> x).toArray());
	}

	@Override
	public Iterator<Double> iterator() {
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

	public double get(int index) {
		double output;
		if (!clone) {
			output = data[index];
		} else {
			output = cloneOf.get(minorOf[index]);
		}
		return appliedFunction.apply(output * factorOf);
	}

	@Override
	public double dot(Vector<S> anotherVector) {
		double output = 0;
		for (int i = 0; i < assertSizeGetMinimum(anotherVector); i++) {
			output += anotherVector.get(i) * this.get(i);
		}
		return output;
	}

	public void set(int index, double value) {
		if (clone) {
			this.data = cloneOf.data;
			this.clone = false;
			this.factorOf *= cloneOf.factorOf;
			this.cloneOf = null; // frees up the garbage collector
		}
		data[index] = value;
	}

	@Override
	public void plus(Vector<S> anotherVector) {
		for (int i = 0; i < assertSizeGetMinimum(anotherVector); i++) {
			data[i] += anotherVector.get(i);
		}
	}

	@Override
	public void minus(Vector<S> anotherVector) {
		for (int i = 0; i < assertSizeGetMinimum(anotherVector); i++) {
			data[i] -= anotherVector.get(i);
		}
	}

	@Override
	public void divide(double factor) {
		this.factorOf /= factor;
	}

	@Override
	public void multiple(double factor) {
		this.factorOf *= factor;
	}

	@Override
	public void invert() {
		this.factorOf *= -1;
	}

	public int getSize() {
		return data.length;
	}

	public Vector<S> unit() {
		GBBasisVector<S> clonedVector = this.clone();
		clonedVector.divide(norm());
		return clonedVector;
	}

	public double norm() {
		return Math
			.sqrt(StreamSupport.stream(this.spliterator(), false).map(x -> Math.pow(x, 2)).reduce(Double::sum).orElseGet(() -> Double.NaN));
	}

	public GBBasisVector<S> getMinor(int[] minorOf) {
		return new GBBasisVector<S>(this, true, minorOf);
	}

	public double assertSizeGetMinimum(Vector<?> anotherVector) {
		int theirSize = anotherVector.getSize();
		int mySize = this.getSize();
		if (mySize != theirSize) {
			AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.ERROR, "GotBadVectorSizeExpected" + mySize + "Got" + theirSize, () -> true));
		}
		return Math.min(theirSize, mySize);
	}

	public void injectFunctionOnOutput(Function<Double, Double> injectedFunction) {
		this.appliedFunction = injectedFunction;
	}

	@Override
	public GBBasisVector<S> clone() {
		GBBasisVector<S> cloned;
		try {
			cloned = (GBBasisVector<S>) super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException(e);
		}
		cloned.appliedFunction = this.appliedFunction;
		this.cloneOf = this;
		this.clone = true;
		this.data = null;
		this.factorOf = 1;
		return new GBBasisVector<>(this, true);
	}

}
