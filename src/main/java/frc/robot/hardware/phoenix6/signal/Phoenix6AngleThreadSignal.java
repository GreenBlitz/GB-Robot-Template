package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.AngleArraySignal;
import frc.robot.subsystems.swerve.OdometryThread;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;

public class Phoenix6AngleThreadSignal extends AngleArraySignal {

	private final Queue<TimedValue<Double>> threadTimedValues;
	private final String name;

	protected Phoenix6AngleThreadSignal(String name, StatusSignal<?> statusSignal, AngleUnit angleUnit, OdometryThread thread) {
		super(statusSignal.getName(), angleUnit);
		this.threadTimedValues = thread.addSignal(statusSignal);
		this.name = name;
	}

	@Override
	protected TimedValue<Rotation2d>[] updateValues(TimedValue<Rotation2d>[] timedValues) {
		timedValues = new TimedValue[threadTimedValues.size()];

		for (int i = 0; i < timedValues.length; i++) {
			OdometryThread.THREAD_LOCK.lock();
			try {
				TimedValue<Double> value = threadTimedValues.poll();
				timedValues[i] = new TimedValue<>(angleUnit.toRotation2d(value.getValue()), value.getTimestamp());
				System.out.println(name);
			} finally {
				OdometryThread.THREAD_LOCK.unlock();
			}
		}
		return timedValues;
	}
	
	@Override
	public void print() {
		System.out.println(name+ "THIS ONE");
	}
}
