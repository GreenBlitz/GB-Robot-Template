package frc.utils;


public class PIDObject {

	private int slot;

	private double p, i, d, feedforward;

	private double derivativeFilter;

	private double errorZoneForIToAccumulate;

	private double maxPower;

	private double minPower;

	public PIDObject() {
		this.p = 0;
		this.d = 0;
		this.i = 0;
		this.feedforward = 0;
		this.derivativeFilter = 0;
		this.errorZoneForIToAccumulate = 0;
		this.maxPower = 1;
		this.minPower = -1;
	}

	public PIDObject(PIDObject other) {
		this.slot = other.slot;
		this.p = other.p;
		this.d = other.d;
		this.i = other.i;
		this.feedforward = other.feedforward;
		this.derivativeFilter = other.derivativeFilter;
		this.errorZoneForIToAccumulate = other.errorZoneForIToAccumulate;
		this.maxPower = other.maxPower;
		this.minPower = other.minPower;
	}

	public int getSlot(){
		return slot;
	}

	public double getP() {
		return p;
	}

	public double getD() {
		return d;
	}

	public double getI() {
		return i;
	}

	public double getFeedforward() {
		return feedforward;
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

	public PIDObject withSlot(int slot){
		this.slot = slot;
		return this;
	}

	public PIDObject withP(double p) {
		this.p = p;
		return this;
	}

	public PIDObject withI(double i) {
		this.i = i;
		return this;
	}

	public PIDObject withD(double d) {
		this.d = d;
		return this;
	}

	public PIDObject withFeedForward(double feedforward) {
		this.feedforward = feedforward;
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
