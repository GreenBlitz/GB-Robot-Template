package frc.utils.gbmeth;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N2;

import java.util.Collection;

public class GBVector2D extends GBVector<N2> {

	public GBVector2D(double... data) {
		super(data);
	}

	public GBVector2D(GBVector<N2> vector) {
		super(vector);
	}

	public <E extends Num> GBVector2D(edu.wpi.first.math.Vector<E> vector) {
		super(vector.getData());
	}

	public GBVector2D(Collection<Double> data) {
		super(data);
	}

	public GBVector2D(double angle, double distance) {
		super(new double[] {Math.cos(angle), Math.sin(angle)});
		this.timesBy(distance);
	}

	public double getAngle() {
		return Math.atan2(this.get(1), this.get(0));
	}

}
