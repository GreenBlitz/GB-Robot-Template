package edu.greenblitz.robotName.utils;

import edu.wpi.first.math.controller.PIDController;

public class PIDObject {

    private double kp, kd, ki, ff;

    private double iZone;

    private double tolerance;

    private int inverted = 1;

    private double maxPower;

    public PIDObject() {
        this(0);
    }

    public PIDObject(double kp, double ki, double kd, double kf, int inv) {
        this(kp, ki, kd, kf, inv, 0);
    }

    public PIDObject(double kp, double ki, double kd, double kf, int inv, double iZone) {
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
        this.ff = kf;
        this.iZone = iZone;
        setInverted(inv);
    }

    public PIDObject(PIDObject other) {
        this.kp = other.kp;
        this.kd = other.kd;
        this.ki = other.ki;
        this.ff = other.ff;
        this.iZone = other.iZone;
        this.tolerance = other.tolerance;
        this.inverted = other.inverted;
        this.maxPower = other.maxPower;
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

    public PIDObject(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 1);
    }

    public PIDObject(double kp, double ki, double kd, int inv) {
        this(kp, ki, kd, 0.0, inv);
    }

    public PIDObject(double kp, double ki, int inv) {
        this(kp, ki, 0.0, inv);
    }

    public PIDObject(double kp, int inv) {
        this(kp, 0.0, inv);
    }

    @Override
    public String toString() {
        return "PIDObject{" +
                "kp=" + kp +
                ", kd=" + kd +
                ", ki=" + ki +
                ", kf=" + ff +
                ", inv=" + inverted +
                ", iZone=" + iZone + "}";
    }

    public void invert() {
        inverted *= -1;
    }

    public int getInverted() {
        return inverted;
    }

    public void setInverted(int value) {
        inverted = value >= 0 ? 1 : -1;
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

    public double getKf() {
        return ff;
    }

    public double getIZone() {
        return iZone;
    }

    public void setIZone(double iZone) {
        this.iZone = iZone;
    }

    public void setFF(double ff) {
        this.ff = ff;
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
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

    public PIDObject withTolerance(double tolerance) {
        setTolerance(tolerance);
        return this;
    }

    public PIDObject withMaxPower(double maxPower) {
        setMaxPower(maxPower);
        return this;
    }

    public PIDController getPIDController() {
        PIDController pidController = new PIDController(this.kp, this.ki, this.kd);
        pidController.setTolerance(this.tolerance);
        return pidController;
    }
}