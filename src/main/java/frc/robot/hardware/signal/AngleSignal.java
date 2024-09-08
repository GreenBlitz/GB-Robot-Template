package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Arrays;
import java.util.function.DoubleFunction;

public abstract class AngleSignal extends InputSignal<Rotation2d> {

    private final DoubleFunction<Rotation2d> toAngle;

    protected AngleSignal(DoubleFunction<Rotation2d> toAngle) {
        super(new Rotation2d());
        this.toAngle = toAngle;
    }

    protected void setNewValues(double[] newValues) {
        setNewValues(Arrays.stream(newValues).mapToObj(toAngle).toArray(Rotation2d[]::new));
    }

    protected void setNewValues(double newValue) {
        setNewValues(toAngle.apply(newValue));
    }

}