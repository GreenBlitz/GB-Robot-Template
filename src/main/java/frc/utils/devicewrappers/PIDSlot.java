package frc.utils.devicewrappers;

import com.revrobotics.SparkPIDController.AccelStrategy;

public class PIDSlot {

    private int slot;
    private double kP;
    private double kI;
    private double kD;
    private double FF;
    private double iZone;
    private double outputRangeMax;
    private double outputRangeMin;
    private double smartMotionMaxVelocity;
    private double smartMotionMaxAcceleration;
    private double smartMotionMinOutputVelocity;
    private double smartMotionAllowedClosedLoopError;
    private AccelStrategy smartMotionAccelStrategy;
    private double iMaxAccumulator;
    private double iAccumulator;
    private boolean positionPIDWrappingEnabled;
    private double positionPIDWrappingMinInput;
    private double positionPIDWrappingMaxInput;

    public PIDSlot(){

    }

    public int getSlot() {
        return slot;
    }

    public void setSlot(int slot) {
        this.slot = slot;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getFF() {
        return FF;
    }

    public void setFF(double FF) {
        this.FF = FF;
    }

    public double getiZone() {
        return iZone;
    }

    public void setiZone(double iZone) {
        this.iZone = iZone;
    }

    public double getOutputRangeMax() {
        return outputRangeMax;
    }

    public void setOutputRangeMax(double outputRangeMax) {
        this.outputRangeMax = outputRangeMax;
    }

    public double getOutputRangeMin() {
        return outputRangeMin;
    }

    public void setOutputRangeMin(double outputRangeMin) {
        this.outputRangeMin = outputRangeMin;
    }

    public double getSmartMotionMaxVelocity() {
        return smartMotionMaxVelocity;
    }

    public void setSmartMotionMaxVelocity(double smartMotionMaxVelocity) {
        this.smartMotionMaxVelocity = smartMotionMaxVelocity;
    }

    public double getSmartMotionMaxAcceleration() {
        return smartMotionMaxAcceleration;
    }

    public void setSmartMotionMaxAcceleration(double smartMotionMaxAcceleration) {
        this.smartMotionMaxAcceleration = smartMotionMaxAcceleration;
    }

    public double getSmartMotionMinOutputVelocity() {
        return smartMotionMinOutputVelocity;
    }

    public void setSmartMotionMinOutputVelocity(double smartMotionMinOutputVelocity) {
        this.smartMotionMinOutputVelocity = smartMotionMinOutputVelocity;
    }

    public double getSmartMotionAllowedClosedLoopError() {
        return smartMotionAllowedClosedLoopError;
    }

    public void setSmartMotionAllowedClosedLoopError(double smartMotionAllowedClosedLoopError) {
        this.smartMotionAllowedClosedLoopError = smartMotionAllowedClosedLoopError;
    }

    public AccelStrategy getSmartMotionAccelStrategy() {
        return smartMotionAccelStrategy;
    }

    public void setSmartMotionAccelStrategy(AccelStrategy smartMotionAccelStrategy) {
        this.smartMotionAccelStrategy = smartMotionAccelStrategy;
    }

    public double getiMaxAccumulator() {
        return iMaxAccumulator;
    }

    public void setiMaxAccumulator(double iMaxAccumulator) {
        this.iMaxAccumulator = iMaxAccumulator;
    }

    public double getiAccumulator() {
        return iAccumulator;
    }

    public void setiAccumulator(double iAccumulator) {
        this.iAccumulator = iAccumulator;
    }

    public boolean isPositionPIDWrappingEnabled() {
        return positionPIDWrappingEnabled;
    }

    public void setPositionPIDWrappingEnabled(boolean positionPIDWrappingEnabled) {
        this.positionPIDWrappingEnabled = positionPIDWrappingEnabled;
    }

    public double getPositionPIDWrappingMinInput() {
        return positionPIDWrappingMinInput;
    }

    public void setPositionPIDWrappingMinInput(double positionPIDWrappingMinInput) {
        this.positionPIDWrappingMinInput = positionPIDWrappingMinInput;
    }

    public double getPositionPIDWrappingMaxInput() {
        return positionPIDWrappingMaxInput;
    }

    public void setPositionPIDWrappingMaxInput(double positionPIDWrappingMaxInput) {
        this.positionPIDWrappingMaxInput = positionPIDWrappingMaxInput;
    }
}
