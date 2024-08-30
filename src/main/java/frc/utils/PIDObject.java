package frc.utils;


public class PIDObject {

	private double P, I, D, FF;

	private double derivativeFilter;

	private double errorZoneForIToAccumulate;

	private double maxPower;

	private double minPower;

	public PIDObject() {
		this.P = 0;
		this.D = 0;
		this.I = 0;
		this.FF = 0;
		this.derivativeFilter = 0;
		this.errorZoneForIToAccumulate = 0;
		this.maxPower = 1;
		this.minPower = -1;
	}

	public PIDObject(PIDObject other) {
		this.P = other.P;
		this.D = other.D;
		this.I = other.I;
		this.FF = other.FF;
		this.derivativeFilter = other.derivativeFilter;
		this.errorZoneForIToAccumulate = other.errorZoneForIToAccumulate;
		this.maxPower = other.maxPower;
		this.minPower = other.minPower;
	}

	public double getP() {
		return P;
	}

	public double getD() {
		return D;
	}

	public double getI() {
		return I;
	}

	public double getFF() {
		return FF;
	}

	public double getIZone() {
		return errorZoneForIToAccumulate;
	}

	public double getMaxPower() {
		return maxPower;
	}

	public double getMinPower() {
		return minPower;
	}

	public double getDFilter() {
		return derivativeFilter;
	}

	public PIDObject withP(double P) {
		this.P = P;
		return this;
	}

	public PIDObject withI(double I) {
		this.I = I;
		return this;
	}

	public PIDObject withD(double D) {
		this.D = D;
		return this;
	}

	public PIDObject withFF(double FF) {
		this.FF = FF;
		return this;
	}

	public PIDObject withDFilter(double dFilter) {
		this.derivativeFilter = dFilter;
		return this;
	}

	public PIDObject withIZone(double errorZoneForIToAccumulate) {
		this.errorZoneForIToAccumulate = errorZoneForIToAccumulate;
		return this;
	}

	public PIDObject withMaxPower(double maxPower) {
		this.maxPower = maxPower;
		return this;
	}

	public PIDObject withMinPower(double minPower) {
		this.minPower = minPower;
		return this;
	}

}
