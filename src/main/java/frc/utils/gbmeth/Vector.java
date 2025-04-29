package frc.utils.gbmeth;

public interface Vector<Num> extends Cloneable {

	double dot(Vector<Num> anotherVector);

	void set(int index, double value);

	double get(int index);

	void plus(Vector<Num> anotherVector);

	void minus(Vector<Num> anotherVector);

	void divide(double factor);

	void multiple(double factor);

	void invert();

	int size();

	Vector<Num> clone();

	Vector<Num> unit();

	double norm();

}
