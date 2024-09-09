package frc.robot.hardware.newsignal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.AngleUnit;
import org.littletonrobotics.junction.LogTable;

public abstract class AngleSignal implements InputSignal<Rotation2d> {

	private final String name;
	private final AngleUnit angleUnit;
	private Rotation2d value;

	public AngleSignal(String name, AngleUnit angleUnit) {
		this.name = name;
        this.angleUnit = angleUnit;
		this.value = new Rotation2d();
	}

	@Override
	public Rotation2d getLatestValue() {
		return value;
	}

	@Override
	public Rotation2d[] asArray() {
		return new Rotation2d[] {value};
	}

	@Override
	public void toLog(LogTable table) {
		value = angleUnit.toAngle(getNewValue());
		table.put(name, value);
	}

	@Override
	public void fromLog(LogTable table) {
		value = table.get(name, new Rotation2d());
	}

	protected abstract double getNewValue();

}
