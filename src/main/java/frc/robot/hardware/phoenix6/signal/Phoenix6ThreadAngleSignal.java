package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.ArrayAngleSignal;
import frc.robot.subsystems.swerve.odometrythread.OdometryThread;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;

import java.util.Collection;
import java.util.Queue;

public class Phoenix6ThreadAngleSignal extends ArrayAngleSignal implements SignalGetter {

	private final Queue<TimedValue<Double>> threadTimedValues;
	private final OdometryThread thread;
	private final StatusSignal<?> statusSignal;

	protected Phoenix6ThreadAngleSignal(StatusSignal<?> statusSignal, AngleUnit angleUnit, OdometryThread thread) {
		super(statusSignal.getName(), angleUnit);
		this.thread = thread;
		this.statusSignal = statusSignal;
		this.threadTimedValues = thread.addSignal(statusSignal);
	}

	public void addLatencyCompensation(Phoenix6ThreadAngleSignal slopeSignal) {
		thread.addLatencyAndSlopeSignals(statusSignal, threadTimedValues, slopeSignal.getSignal());
	}

	@Override
	protected void updateValues(Collection<TimedValue<Rotation2d>> timedValues) {
		thread.threadQueuesLock.lock();
		try {
			timedValues.clear();
			timedValues.addAll(
				threadTimedValues.stream()
					.map(timedValue -> new TimedValue<>(angleUnit.toRotation2d(timedValue.getValue()), timedValue.getTimestamp()))
					.toList()
			);
			threadTimedValues.clear();
		} finally {
			thread.threadQueuesLock.unlock();
		}
	}

	@Override
	public StatusSignal<?> getSignal() {
		return statusSignal;
	}

}
