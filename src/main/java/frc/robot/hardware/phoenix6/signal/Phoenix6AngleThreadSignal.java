package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.AngleArraySignal;
import frc.robot.subsystems.swerve.OdometryThread;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;

import java.util.Queue;

public class Phoenix6AngleThreadSignal extends AngleArraySignal {

	private final Queue<TimedValue<Double>> threadTimedValues;

	protected Phoenix6AngleThreadSignal(StatusSignal<?> statusSignal, AngleUnit angleUnit, OdometryThread thread) {
		super(statusSignal.getName(), angleUnit);
		this.threadTimedValues = thread.addSignal(statusSignal);
	}

	@Override
	protected TimedValue<Rotation2d>[] updateValues(TimedValue<Rotation2d>[] timedValues) {
		timedValues = new TimedValue[threadTimedValues.size()];

		for (int i = 0; i < timedValues.length; i++) {
			TimedValue<Double> value = threadTimedValues.poll();
			timedValues[i] = new TimedValue<>(angleUnit.toRotation2d(value.getValue()), value.getTimestamp());
		}
		return timedValues;
	}

}
