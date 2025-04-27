package frc.utils.gbmeth;

import edu.wpi.first.math.Num;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;

import java.util.*;
import java.util.function.Function;
import java.util.stream.StreamSupport;

public final class GBBasisVector<S extends Num> implements Cloneable, Iterable<Double>, Vector<S> {

	private GBBasisVector<?> cloneOf;
	private boolean clone;
	private double[] data;
	private double factorOf;
	private Function<Double, Double> appliedFunction;

	private GBBasisVector(GBBasisVector<?> anotherVector) {
		this.cloneOf = anotherVector;
		this.clone = true;
		this.data = null;
		this.factorOf = 1;
		this.appliedFunction = anotherVector.appliedFunction;
	}

	public GBBasisVector(double[] data) {
		this.data = data;
		this.clone = false;
		this.factorOf = 1;
		this.cloneOf = null;
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
			output = cloneOf.get(index);
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
	public String toString() {
		StringBuilder output = new StringBuilder();
		for (double x : this) {
			output.append(String.format("%.4f", x));
		}
		return output.toString();
	}

	public String debugString() {
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
	public GBBasisVector<S> clone() {
		GBBasisVector<S> cloned;
		try {
			cloned = (GBBasisVector<S>) super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException(e);
		}
		cloned.appliedFunction = this.appliedFunction;
		this.cloneOf = clone ? this.cloneOf.clone() : null;
		this.clone = true;
		this.data = null;
		this.factorOf = 1;
		return new GBBasisVector<>(this);
	}

}
