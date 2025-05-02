package frc.utils.gbmeth;

public interface Vector<Num> extends Cloneable {

	double dot(Vector<Num> anotherVector);

	void set(int index, double value);

	double get(int index);

	void plusBy(Vector<Num> anotherVector);

	void minusBy(Vector<Num> anotherVector);

	void divideBy(double factor);

	void multipleBy(double factor);

	void invert();

	int size();

	Vector<Num> clone();

	Vector<Num> unit();

	double norm();

	default void timesBy(double factorOf) {
		multipleBy(factorOf);
	}

}
