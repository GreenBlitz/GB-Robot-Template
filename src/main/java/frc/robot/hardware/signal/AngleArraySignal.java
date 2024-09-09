package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.LogTable;

import java.util.Arrays;

public abstract class AngleArraySignal implements InputSignal<Rotation2d> {

    private final String name;
    private final AngleUnit angleUnit;
    private Rotation2d[] values;

    public AngleArraySignal(String name, AngleUnit angleUnit) {
        this.name = name;
        this.angleUnit = angleUnit;
        this.values = new Rotation2d[]{};
    }

    @Override
    public Rotation2d getLatestValue() {
        return values[values.length - 1];
    }

    @Override
    public Rotation2d[] asArray() {
        return values;
    }

    @Override
    public void toLog(LogTable table) {
        values = Arrays.stream(getNewValues()).mapToObj(angleUnit::toAngle).toArray(Rotation2d[]::new);
        table.put(name, values);
    }

    @Override
    public void fromLog(LogTable table) {
        values = table.get(name, new Rotation2d[]{new Rotation2d()});
    }

    protected abstract double[] getNewValues();

}
