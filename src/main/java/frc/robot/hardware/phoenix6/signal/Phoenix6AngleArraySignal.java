package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.AngleArraySignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

import java.util.Queue;

public class Phoenix6AngleArraySignal extends AngleArraySignal {

	private final Queue<?> values;
	private final Queue<?> timestamp;

	protected Phoenix6AngleArraySignal(String name, StatusSignal<?> statusSignal, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.statusSignal = statusSignal;
	}

	@Override
	protected void updateValues(TimedValue<Rotation2d>[] timedValues) {
		timedValues.setValue(angleUnit.toRotation2d(statusSignal.getValueAsDouble()));
		timedValues.setTimestamp(TimeUtil.getCurrentTimeSeconds() - statusSignal.getTimestamp().getLatency());
	}

}
