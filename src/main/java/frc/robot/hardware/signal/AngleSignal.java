package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.AngleUnit;

import java.util.Arrays;
import java.util.function.DoubleFunction;

public abstract class AngleSignal extends InputSignal<Rotation2d> {

	protected final AngleUnit angleUnit;

	protected AngleSignal(AngleUnit angleUnit) {
		super(new Rotation2d());
		this.angleUnit = angleUnit;
	}

	protected void setNewValues(double... newValues) {
		setNewValues(Arrays.stream(newValues).mapToObj(angleUnit::toAngle).toArray(Rotation2d[]::new));
	}

}
