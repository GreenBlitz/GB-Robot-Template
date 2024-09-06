package frc.robot.hardware.signal;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.ctre.Phoenix6SignalsThread;
import org.littletonrobotics.junction.LogTable;

import java.util.Queue;

public class Phoenix6SignalBuilder {

	public interface SignalGetter {

		StatusSignal<Double> getStatusSignal();

	}

	public static Phoenix6Signal registerSignal(StatusSignal<Double> signal, String name) {
		return new Phoenix6Signal(signal.clone(), name);
	}

	public static Phoenix6LatencyBothSignal registerLatencySignal(StatusSignal<Double> signal, StatusSignal<Double> signalSlope, String name) {
		return new Phoenix6LatencyBothSignal(signal.clone(), signalSlope.clone(), name);
	}

	public static Phoenix6LatencySignal registerLatencySignal(StatusSignal<Double> signal, SignalGetter signalSlope, String name) {
		return new Phoenix6LatencySignal(signal.clone(), signalSlope.getStatusSignal(), name);
	}

	public static Phoenix6ThreadSignal registerThreadSignal(ParentDevice parentDevice, StatusSignal<Double> signal, String name) {
		return new Phoenix6ThreadSignal(Phoenix6SignalsThread.getInstance().registerRegularSignal(parentDevice, signal.clone()), name);
	}

	//@formatter:off
    public static Phoenix6ThreadSignal registerThreadSignal(ParentDevice parentDevice, StatusSignal<Double> signal, StatusSignal<Double> signalSlope, String name) {
		return new Phoenix6ThreadSignal(Phoenix6SignalsThread.getInstance().registerLatencySignal(parentDevice, signal.clone(), signalSlope.clone()), name);
	}
    //@formatter:on

	public static class Phoenix6Signal extends InputSignal implements SignalGetter {

		private final StatusSignal<Double> statusSignal;
		private final String name;

		private Phoenix6Signal(StatusSignal<Double> signal, String name) {
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

	public static class Phoenix6LatencySignal extends InputSignal implements SignalGetter {

		private final StatusSignal<Double> statusSignal;
		protected final StatusSignal<Double> signalSlope;
		private final String name;

		private Phoenix6LatencySignal(StatusSignal<Double> signal, StatusSignal<Double> signalSlope, String name) {
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

		private Phoenix6LatencyBothSignal(StatusSignal<Double> signal, StatusSignal<Double> signalSlope, String name) {
			super(signal, signalSlope, name);
		}

		public StatusSignal<Double> getSlopeStatusSignal() {
			// For using refresh all with more signals...
			return signalSlope;
		}

	}

	public static class Phoenix6ThreadSignal extends InputSignal {

		private final Queue<Double> signalQueue;
		private final String name;

		private Phoenix6ThreadSignal(Queue<Double> signalQueue, String name) {
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
