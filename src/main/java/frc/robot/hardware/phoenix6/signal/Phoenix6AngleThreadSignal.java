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
	private final OdometryThread thread;
	private final StatusSignal<?> statusSignal;

	protected Phoenix6AngleThreadSignal(StatusSignal<?> statusSignal, AngleUnit angleUnit, OdometryThread thread) {
		super(statusSignal.getName(), angleUnit);
		this.thread = thread;
		this.statusSignal = statusSignal;
		this.threadTimedValues = thread.addSignal(statusSignal);
	}

	public void addWithLatencyCompensation(Phoenix6AngleThreadSignal slopeSignal) {
		thread.addLatencyAndSlopeSignals(statusSignal, threadTimedValues, slopeSignal.getStatusSignal());
	}

	public StatusSignal<?> getStatusSignal() {
		return statusSignal;
	}

	@Override
	protected TimedValue<Rotation2d>[] updateValues(TimedValue<Rotation2d>[] timedValues) {
		thread.ThreadQueuesLock.lock();
		try {
			timedValues = new TimedValue[threadTimedValues.size()];
		} finally {
			thread.ThreadQueuesLock.unlock();
		}

		for (int i = 0; i < timedValues.length; i++) {
			thread.ThreadQueuesLock.lock();
			try {
				TimedValue<Double> value = threadTimedValues.poll();
				timedValues[i] = new TimedValue<>(angleUnit.toRotation2d(value.getValue()), value.getTimestamp());
			} finally {
				thread.ThreadQueuesLock.unlock();
			}
		}
		return timedValues;
	}

}
