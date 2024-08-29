package frc.utils;


public class PIDObject {

	private double kp, ki, kd, ff;

	private double dFilter;

	private double iZone;

	private double maxPower;

	private boolean empty;

	public PIDObject() {
		this.kp = 0;
		this.kd = 0;
		this.ki = 0;
		this.ff = 0;
		this.dFilter = 0;
		this.iZone = 0;
		this.maxPower = 1;
		this.empty = isEmpty();
	}

	public PIDObject(PIDObject other) {
		this.kp = other.kp;
		this.kd = other.kd;
		this.ki = other.ki;
		this.ff = other.ff;
		this.dFilter = other.dFilter;
		this.iZone = other.iZone;
		this.maxPower = other.maxPower;
		this.empty = other.empty;
	}

	public boolean isEmpty() {
		if (kp != 0) {
			empty = false;
		} else if (ki != 0) {
			empty = false;
		} else if (kd != 0) {
			empty = false;
		} else if (ff != 0) {
			empty = false;
		} else if (dFilter != 0) {
			empty = false;
		} else if (iZone != 0) {
			empty = false;
		} else if (maxPower != 0) {
			empty = false;
		}
		return empty;
	}

	public double getKp() {
		return kp;
	}

	public double getKd() {
		return kd;
	}

	public double getKi() {
		return ki;
	}

	public double getKff() {
		return ff;
	}

	public double getIZone() {
		return iZone;
	}

	public double getMaxPower() {
		return maxPower;
	}

	public double getDFilter() {
		return dFilter;
	}

	public PIDObject withKp(double kp) {
		this.kp = kp;
		isEmpty();
		return this;
	}

	public PIDObject withKi(double ki) {
		this.ki = ki;
		isEmpty();
		return this;
	}

	public PIDObject withKd(double kd) {
		this.kd = kd;
		isEmpty();
		return this;
	}

	public PIDObject withFF(double ff) {
		this.ff = ff;
		isEmpty();
		return this;
	}

	public PIDObject withDFilter(double dFilter) {
		this.dFilter = dFilter;
		isEmpty();
		return this;
	}

	public PIDObject withIZone(double iZone) {
		this.iZone = iZone;
		isEmpty();
		return this;
	}

	public PIDObject withMaxPower(double maxPower) {
		this.maxPower = maxPower;
		isEmpty();
		return this;
	}

}
