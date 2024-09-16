package frc.robot.hardware.signal.phoenix;

import frc.robot.hardware.signal.AngleArraySignal;
import frc.utils.AngleUnit;

import java.util.Queue;

public class Phoenix6ThreadSignal extends AngleArraySignal {

    private final Queue<Double> signalQueue;

    public Phoenix6ThreadSignal(Queue<Double> signalQueue, String name, AngleUnit angleUnit) {
        super(name, angleUnit);
        this.signalQueue = signalQueue;
    }

    @Override
    protected double[] getNewValues() {
        Phoenix6Thread.LOCK.lock();
        try {
            double[] values = signalQueue.stream().mapToDouble(Double::doubleValue).toArray();
            signalQueue.clear();
            return values;
        } finally {
            Phoenix6Thread.LOCK.unlock();
        }
    }

}
