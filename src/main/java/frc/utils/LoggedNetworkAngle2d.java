package frc.utils;

import frc.utils.angle2d.Angle2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LoggedNetworkAngle2d extends LoggedNetworkInput {

	private final String key;
	private final DoubleEntry entry;
	private Angle2d defaultAngle = Angle2d.ZERO;
	private Angle2d angle;

	public LoggedNetworkAngle2d(String key) {
		this.key = key;
		this.entry = NetworkTableInstance.getDefault().getDoubleTopic(key + "InDegrees").getEntry(0.0);
		this.angle = defaultAngle;
		Logger.registerDashboardInput(this);
	}

	public LoggedNetworkAngle2d(String key, Angle2d defaultAngle) {
		this(key);
		setDefault(defaultAngle);
		this.angle = defaultAngle;
	}

	public void setDefault(Angle2d defaultValue) {
		this.defaultAngle = defaultValue;
		entry.set(entry.get(defaultValue.getDegrees()));
	}

	public void set(Angle2d value) {
		entry.set(value.getDegrees());
	}

	public Angle2d get() {
		return angle;
	}

	private final LoggableInputs inputs = new LoggableInputs() {

		public void toLog(LogTable table) {
			table.put(removeSlash(key), angle);
		}

		public void fromLog(LogTable table) {
			angle = table.get(removeSlash(key + "InDegrees"), defaultAngle);
		}

	};

	public void periodic() {
		if (!Logger.hasReplaySource()) {
			angle = Angle2d.fromDegrees(entry.get());
		}
		Logger.processInputs(prefix, inputs);
	}

}
