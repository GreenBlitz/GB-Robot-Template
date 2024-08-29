package frc.utils;


public class PIDObject {

	private double kp, ki, kd, ff;

	private double dFilter;

	private double iZone;

	private double maxPower;

	public PIDObject() {
		this(0);
	}

	public PIDObject(double kp, double ki, double kd, double kff, double dFilter, double iZone, double maxPower) {
		this.kp = kp;
		this.kd = kd;
		this.ki = ki;
		this.ff = kff;
		this.dFilter = dFilter;
		this.iZone = iZone;
		this.maxPower = maxPower;
	}

	public PIDObject(double kp, double ki, double kd, double kff, double dFilter, double iZone) {
		this(kp, ki, kd, kff, dFilter, iZone, 1);
	}

	public PIDObject(double kp, double ki, double kd, double kff, double iZone) {
		this(kp, ki, kd, kff, 0, iZone);
	}

	public PIDObject(double kp, double ki, double kd, double kff) {
		this(kp, ki, kd, kff, 0);
	}

	public PIDObject(double kp, double ki, double kd) {
		this(kp, ki, kd, 0.0);
	}

	public PIDObject(double kp, double ki) {
		this(kp, ki, 0.0);
	}

	public PIDObject(double kp) {
		this(kp, 0.0);
	}

	public PIDObject(PIDObject other) {
		this.kp = other.kp;
		this.kd = other.kd;
		this.ki = other.ki;
		this.ff = other.ff;
		this.dFilter = other.dFilter;
		this.iZone = other.iZone;
		this.maxPower = other.maxPower;
	}

	public double getKp() {
		return kp;
	}

	public void setKp(double kp) {
		this.kp = kp;
	}

	public double getKd() {
		return kd;
	}

	public void setKd(double kd) {
		this.kd = kd;
	}

	public double getKi() {
		return ki;
	}

	public void setKi(double ki) {
		this.ki = ki;
	}

	public double getKff() {
		return ff;
	}

	public void setFF(double ff) {
		this.ff = ff;
	}

	public double getIZone() {
		return iZone;
	}

	public void setIZone(double iZone) {
		this.iZone = iZone;
	}

	public double getMaxPower() {
		return maxPower;
	}

	public void setMaxPower(double maxPower) {
		this.maxPower = maxPower;
	}

	public double getDFilter() {
		return dFilter;
	}

	public void setDFilter(double dFilter) {
		this.dFilter = dFilter;
	}

	public PIDObject withKp(double kp) {
		this.setKp(kp);
		return this;
	}

	public PIDObject withKi(double ki) {
		this.setKi(ki);
		return this;
	}

	public PIDObject withKd(double kd) {
		this.setKd(kd);
		return this;
	}

	public PIDObject withFF(double ff) {
		this.setFF(ff);
		return this;
	}

	public PIDObject withIZone(double iZone) {
		setIZone(iZone);
		return this;
	}

	public PIDObject withMaxPower(double maxPower) {
		setMaxPower(maxPower);
		return this;
	}

}
