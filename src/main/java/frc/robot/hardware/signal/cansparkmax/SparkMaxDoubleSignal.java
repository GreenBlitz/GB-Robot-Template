package frc.robot.hardware.signal.cansparkmax;

import frc.robot.hardware.signal.DoubleSignal;

import java.util.function.Supplier;

public class SparkMaxDoubleSignal extends DoubleSignal {
    Supplier<Double> doubleSignalSupplier;
    public SparkMaxDoubleSignal(String name, Supplier<Double> doubleSignalSupplier){
        super(name);
        this.doubleSignalSupplier = doubleSignalSupplier;
    }

    @Override
    protected Double getNewValue() {
        return doubleSignalSupplier.get();
    }
}
