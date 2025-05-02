package frc.utils.gbmeth;

public interface Vector<Num> extends Cloneable {

	double dot(Vector<Num> anotherVector);

	void set(int index, double value);

	default Vector<Num> cloneAndSet(int index, double value) {
		Vector<Num> copy = this.deepClone();
		copy.set(index, value);
		return copy;
	}

	double get(int index);

	void plusBy(Vector<Num> anotherVector);

	default Vector<Num> plus(Vector<Num> anotherVector) {
		Vector<Num> copy = this.clone();
		copy.plusBy(anotherVector);
		return copy;
	}

	void minusBy(Vector<Num> anotherVector);

	default Vector<Num> minus(Vector<Num> anotherVector) {
		Vector<Num> copy = this.clone();
		copy.minusBy(anotherVector);
		return copy;
	}

	void divideBy(double factor);

	default Vector<Num> divide(double factor) {
		Vector<Num> copy = this.clone();
		copy.divideBy(factor);
		return copy;
	}

	void multipleBy(double factor);

	default Vector<Num> multiple(double factor) {
		Vector<Num> copy = this.clone();
		copy.multipleBy(factor);
		return copy;
	}

	void invert();

	default Vector<Num> cloneAndInvert() {
		Vector<Num> copy = this.clone();
		copy.invert();
		return copy;
	}

	int size();

	Vector<Num> clone();

	Vector<Num> unit();

	double norm();

	Vector<Num> deepClone();

	default void timesBy(double factorOf) {
		multipleBy(factorOf);
	}

}
