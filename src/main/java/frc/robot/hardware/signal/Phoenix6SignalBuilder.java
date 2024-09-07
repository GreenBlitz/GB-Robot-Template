package frc.robot.hardware.signal;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.ctre.Phoenix6SignalsThread;
import frc.utils.ctre.PhoenixProUtils;
import org.littletonrobotics.junction.LogTable;

import java.util.Queue;
import java.util.function.DoubleFunction;

public class Phoenix6SignalBuilder {

	private static final int UPDATE_FREQUENCY_RETRIES = 5;

	public interface SignalGetter {

		StatusSignal<Double> getStatusSignal();

	}

	private static StatusSignal<Double> cloneWithFrequency(StatusSignal<Double> signal, double frequency) {
		StatusSignal<Double> signalClone = signal.clone();
		PhoenixProUtils.checkWithRetry(() -> signalClone.setUpdateFrequency(frequency), UPDATE_FREQUENCY_RETRIES);
		return signalClone;
	}

	//@formatter:off
	public static Phoenix6DoubleSignal registerSignal(StatusSignal<Double> signal, String name, double frequency) {
		return new Phoenix6DoubleSignal(cloneWithFrequency(signal, frequency), name);
	}

	public static Phoenix6AngleSignal registerSignal(StatusSignal<Double> signal, String name, double frequency, DoubleFunction<Rotation2d> toAngle) {
		return new Phoenix6AngleSignal(cloneWithFrequency(signal, frequency), name, toAngle);
	}

	/**
	 * Use only if not fetching the slope signal!!!
	 */
	public static Phoenix6LatencyBothSignal registerLatencySignal(
			StatusSignal<Double> signal,
			StatusSignal<Double> signalSlope,
			String name,
			double frequency,
			DoubleFunction<Rotation2d> toAngle
	) {
		return new Phoenix6LatencyBothSignal(cloneWithFrequency(signal, frequency), cloneWithFrequency(signalSlope, frequency), name, toAngle);
	}

	public static Phoenix6LatencySignal registerLatencySignal(
			StatusSignal<Double> signal,
			SignalGetter signalSlope,
			String name,
			double frequency,
			DoubleFunction<Rotation2d> toAngle
	) {
		return new Phoenix6LatencySignal(cloneWithFrequency(signal, frequency), signalSlope.getStatusSignal(), name, toAngle);
	}

	public static Phoenix6ThreadSignal registerThreadSignal(
			ParentDevice parentDevice,
			StatusSignal<Double> signal,
			String name,
			double frequency,
			DoubleFunction<Rotation2d> toAngle
	) {
		StatusSignal<Double> signalClone = cloneWithFrequency(signal, frequency);
		return new Phoenix6ThreadSignal(Phoenix6SignalsThread.getInstance().registerRegularSignal(parentDevice, signalClone), name, toAngle);
	}

    public static Phoenix6ThreadSignal registerThreadSignal(
			ParentDevice parentDevice,
			StatusSignal<Double> signal,
			StatusSignal<Double> signalSlope,
			String name,
			double frequency,
			DoubleFunction<Rotation2d> toAngle
	) {
		StatusSignal<Double> signalClone = cloneWithFrequency(signal, frequency);
		StatusSignal<Double> slopeSignalClone = cloneWithFrequency(signalSlope, frequency);
		return new Phoenix6ThreadSignal(
				Phoenix6SignalsThread.getInstance().registerLatencySignal(parentDevice, signalClone, slopeSignalClone),
				name,
				toAngle
		);
	}
    //@formatter:on

	public static class Phoenix6DoubleSignal extends InputSignal.DoubleSignal implements SignalGetter {

		private final StatusSignal<Double> statusSignal;
		private final String name;

		private Phoenix6DoubleSignal(StatusSignal<Double> signal, String name) {
			this.statusSignal = signal;
			this.name = name;
		}

		@Override
		public void toLog(LogTable table) {
			// Must be refreshed before!!!
			double latestValue = statusSignal.getValueAsDouble();
			table.put(name, latestValue);
			setNewValues(latestValue);
		}

		@Override
		public StatusSignal<Double> getStatusSignal() {
			// For using refresh all with more signals...
			return statusSignal;
		}

	}

	public static class Phoenix6AngleSignal extends InputSignal.AngleSignal implements SignalGetter {

		private final StatusSignal<Double> statusSignal;
		private final String name;

		private Phoenix6AngleSignal(StatusSignal<Double> signal, String name, DoubleFunction<Rotation2d> toAngle) {
			super(toAngle);
			this.statusSignal = signal;
			this.name = name;
		}

		@Override
		public void toLog(LogTable table) {
			// Must be refreshed before!!!
			double latestValue = statusSignal.getValueAsDouble();
			table.put(name, latestValue);
			setNewValues(latestValue);
		}

		@Override
		public StatusSignal<Double> getStatusSignal() {
			// For using refresh all with more signals...
			return statusSignal;
		}

	}

	public static class Phoenix6LatencySignal extends InputSignal.AngleSignal implements SignalGetter {

		private final StatusSignal<Double> statusSignal;
		protected final StatusSignal<Double> signalSlope;
		private final String name;

		private Phoenix6LatencySignal(
			StatusSignal<Double> signal,
			StatusSignal<Double> signalSlope,
			String name,
			DoubleFunction<Rotation2d> toAngle
		) {
			super(toAngle);
			this.statusSignal = signal;
			this.signalSlope = signalSlope;
			this.name = name;
		}

		@Override
		public void toLog(LogTable table) {
			// Must be refreshed before!!!
			double latestValue = BaseStatusSignal.getLatencyCompensatedValue(statusSignal, signalSlope);
			table.put(name, latestValue);
			setNewValues(latestValue);
		}

		@Override
		public StatusSignal<Double> getStatusSignal() {
			// For using refresh all with more signals...
			return statusSignal;
		}

	}

	public static class Phoenix6LatencyBothSignal extends Phoenix6LatencySignal implements SignalGetter {

		private Phoenix6LatencyBothSignal(
			StatusSignal<Double> signal,
			StatusSignal<Double> signalSlope,
			String name,
			DoubleFunction<Rotation2d> toAngle
		) {
			super(signal, signalSlope, name, toAngle);
		}

		public StatusSignal<Double> getSlopeStatusSignal() {
			// For using refresh all with more signals...
			return signalSlope;
		}

	}

	public static class Phoenix6ThreadSignal extends InputSignal.AngleSignal {

		private final Queue<Double> signalQueue;
		private final String name;

		private Phoenix6ThreadSignal(Queue<Double> signalQueue, String name, DoubleFunction<Rotation2d> toAngle) {
			super(toAngle);
			this.signalQueue = signalQueue;
			this.name = name;
		}

		@Override
		public void toLog(LogTable table) {
			Swerve.ODOMETRY_LOCK.lock();
			double[] latestValues = signalQueue.stream().mapToDouble(Double::doubleValue).toArray();
			signalQueue.clear();
			Swerve.ODOMETRY_LOCK.unlock();

			table.put(name, latestValues);
			setNewValues(latestValues);
		}

	}

}
