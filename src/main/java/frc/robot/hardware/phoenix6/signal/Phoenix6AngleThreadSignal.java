package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.AngleArraySignal;
import frc.robot.subsystems.swerve.OdometryThread;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

import java.util.Queue;

public class Phoenix6AngleThreadSignal extends AngleArraySignal {

	private final Queue<?> values;
	private final Queue<?> timestamps;

	protected Phoenix6AngleThreadSignal(String name, StatusSignal<?> statusSignal, AngleUnit angleUnit, OdometryThread thread) {
		super(name, angleUnit);
		this.values = thread.addSignal(statusSignal);
		this.timestamps = thread.getTimestamps();
	}

	@Override
	protected void updateValues(TimedValue<Rotation2d>[] timedValues) {
		timedValues.setValue(angleUnit.toRotation2d(statusSignal.getValueAsDouble()));
		timedValues.setTimestamp(TimeUtil.getCurrentTimeSeconds() - statusSignal.getTimestamp().getLatency());
	}

}
