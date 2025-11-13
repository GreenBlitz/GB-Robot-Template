package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class LoggedNetworkRotation2d extends LoggedNetworkInput {

	private final String key;
	private final DoubleEntry entry;
	private Rotation2d defaultValue = Rotation2d.fromDegrees(0);
	private Rotation2d value;

	public LoggedNetworkRotation2d(String key) {
		this.key = key;
		this.entry = NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(0.0);
		this.value = defaultValue;
		Logger.registerDashboardInput(this);
	}

	public LoggedNetworkRotation2d(String key, Rotation2d defaultValue) {
		this(key);
		setDefault(defaultValue);
		this.value = defaultValue;
	}

	public void setDefault(Rotation2d defaultValue) {
		this.defaultValue = defaultValue;
		entry.set(entry.get(defaultValue.getDegrees()));
	}

	public void set(Rotation2d value) {
		entry.set(value.getDegrees());
	}

	public Rotation2d get() {
		return value;
	}

	private final LoggableInputs inputs = new LoggableInputs() {

		public void toLog(LogTable table) {
			table.put(removeSlash(key), value);
		}

		public void fromLog(LogTable table) {
			value = table.get(removeSlash(key), defaultValue);
		}

	};

	public void periodic() {
		if (!Logger.hasReplaySource()) {
			value = Rotation2d.fromDegrees(entry.get());
		}
		Logger.processInputs(prefix, inputs);
	}

}
