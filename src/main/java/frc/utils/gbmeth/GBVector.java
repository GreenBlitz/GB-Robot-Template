package frc.utils.gbmeth;

import edu.wpi.first.math.Num;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.ejml.simple.SimpleMatrix;

import java.util.*;
import java.util.function.Function;
import java.util.stream.IntStream;
import java.util.stream.StreamSupport;

public class GBVector<S extends Num> implements Cloneable, Iterable<Double>, Vector<S> {

	GBVector<?> cloneOfOptional;
	boolean isClone;
	double[] data;
	double factorOf;
	Function<Double, Double> appliedFunction;

	private GBVector(GBVector<?> anotherVector) {
		this.cloneOfOptional = anotherVector;
		this.isClone = true;
		this.data = null;
		this.factorOf = 1;
		this.appliedFunction = anotherVector.appliedFunction;
	}

	public GBVector(double... data) {
		this.data = data;
		this.isClone = false;
		this.factorOf = 1;
		this.cloneOfOptional = null;
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
		if (!isClone) {
			output = data[index];
		} else {
			output = cloneOfOptional.get(index);
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
		if (isClone) {
			this.data = cloneOfOptional.data.clone();
			this.isClone = false;
			this.factorOf *= cloneOfOptional.factorOf;
			this.cloneOfOptional = null; // frees up the garbage collector
		}
		data[index] = value;
	}

	@Override
	public void plusBy(Vector<S> anotherVector) {
		for (int i = 0; i < assertSizeGetMinimum(anotherVector); i++) {
			this.set(i, this.get(i) + (anotherVector.get(i) / factorOf));
		}
	}

	@Override
	public void minusBy(Vector<S> anotherVector) {
		for (int i = 0; i < assertSizeGetMinimum(anotherVector); i++) {
			this.set(i, this.get(i) - (anotherVector.get(i) / factorOf));
		}
	}

	@Override
	public final void divideBy(double factor) {
		this.factorOf /= factor;
	}

	@Override
	public final void multipleBy(double factor) {
		this.factorOf *= factor;
	}

	@Override
	public final void invert() {
		this.multipleBy(-1);
	}

	public final int size() {
		return data.length;
	}

	public final Vector<S> unit() {
		GBVector<S> clonedVector = this.clone();
		clonedVector.divideBy(norm());
		return clonedVector;
	}

	public final double norm() {
		return Math
			.sqrt(StreamSupport.stream(this.spliterator(), false).map(x -> Math.pow(x, 2)).reduce(Double::sum).orElseGet(() -> Double.NaN));
	}

	public final double angleBetween(Vector<S> anotherVector) {
		return Math.acos(cosineBetween(anotherVector));
	}

	public final double cosineBetween(Vector<S> anotherVector) {
		return this.dot(anotherVector) / (this.norm() * anotherVector.norm());
	}

	public final double sineBetween(Vector<S> anotherVector) {
		return Math.sqrt(1 - Math.pow(cosineBetween(anotherVector), 2));
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
		output.append("(");
		for (double x : this) {
			output.append(String.format("%.4f", x));
			output.append(", ");
		}
		output.append("\b\b)");
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
			+ cloneOfOptional
			+ ", clone="
			+ isClone
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
		this.cloneOfOptional = isClone ? this.cloneOfOptional.clone() : null;
		this.isClone = true;
		this.data = null;
		this.factorOf = 1;
		return new GBVector<>(this);
	}

	public static <E extends Num> GBVector<E> deepClone(GBVector<E> vector) {
		GBVector<E> cloned = vector.clone();
		cloned.data = vector.data.clone();
		return cloned;
	}

	@Override
	public boolean equals(Object o) {
		if (!(o instanceof GBVector<?> gbVector))
			return false;
		if (gbVector.isClone || this.isClone) {
			if (this.size() != gbVector.size())
				return false;
			return IntStream.range(0, gbVector.size()).allMatch(index -> Double.compare(gbVector.get(index), gbVector.get(index)) == 0);
		} else {
			return Double.compare(factorOf, gbVector.factorOf) == 0 && Objects.deepEquals(data, gbVector.data);
		}
	}

	@Override
	public int hashCode() {
		if (this.isClone) {
			return Objects.hash(Arrays.hashCode(data), factorOf);
		} else {
			return this.cloneOfOptional.hashCode(); // this recursion will end
		}
	}

}
