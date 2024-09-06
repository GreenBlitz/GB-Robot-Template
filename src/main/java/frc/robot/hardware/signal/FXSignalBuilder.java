package frc.robot.hardware.signal;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.subsystems.swerve.phoenix6signalsthread.Phoenix6SignalsThread;
import org.littletonrobotics.junction.LogTable;

import java.util.Queue;

public class FXSignalBuilder {

	public static FXSignal registerSignal(StatusSignal<Double> signal, String name) {
		return new FXSignal(signal.clone(), name);
	}

	public static FXLatencySignal registerLatencySignal(StatusSignal<Double> signal, StatusSignal<Double> signalSlope, String name) {
		return new FXLatencySignal(signal.clone(), signalSlope.clone(), name);
	}

	public static FXThreadSignal registerThreadSignal(ParentDevice parentDevice, StatusSignal<Double> signal, String name) {
		return new FXThreadSignal(Phoenix6SignalsThread.getInstance().registerRegularSignal(parentDevice, signal.clone()), name);
	}

	//@formatter:off
    public static FXThreadSignal registerThreadSignal(ParentDevice parentDevice, StatusSignal<Double> signal, StatusSignal<Double> signalSlope, String name) {
		return new FXThreadSignal(Phoenix6SignalsThread.getInstance().registerLatencySignal(parentDevice, signal.clone(), signalSlope.clone()), name);
	}
    //@formatter:on

	public static class FXSignal extends SignalInput {

		private final StatusSignal<Double> statusSignal;
		private final String name;

		private FXSignal(StatusSignal<Double> signal, String name) {
			this.statusSignal = signal;
			this.name = name;
		}

		@Override
		public void toLog(LogTable table) {
			// Must be refreshed before!!!
			table.put(name, statusSignal.getValueAsDouble());
		}

		public BaseStatusSignal getStatusSignal() {
			// For using refresh all with more signals...
			return statusSignal;
		}

	}

	public static class FXLatencySignal extends SignalInput {

		private final StatusSignal<Double> statusSignal;
		private final StatusSignal<Double> signalSlope;
		private final String name;

		private FXLatencySignal(StatusSignal<Double> signal, StatusSignal<Double> signalSlope, String name) {
			this.statusSignal = signal;
			this.signalSlope = signalSlope;
			this.name = name;
		}

		@Override
		public void toLog(LogTable table) {
			// Must be refreshed before!!!
			table.put(name, BaseStatusSignal.getLatencyCompensatedValue(statusSignal, signalSlope));
		}

		public BaseStatusSignal getStatusSignal() {
			// For using refresh all with more signals...
			return statusSignal;
		}

		public BaseStatusSignal getStatusSignalSlope() {
			// For using refresh all with more signals...
			return signalSlope;
		}

	}

	public static class FXThreadSignal extends SignalInput {

		private final Queue<Double> signalQueue;
		private final String name;

		private FXThreadSignal(Queue<Double> signalQueue, String name) {
			this.signalQueue = signalQueue;
			this.name = name;
		}

		@Override
		public void toLog(LogTable table) {
			Phoenix6SignalsThread.SIGNALS_LOCK.lock();
			table.put(name, signalQueue.stream().mapToDouble(Double::doubleValue).toArray());
			signalQueue.clear();
			Phoenix6SignalsThread.SIGNALS_LOCK.unlock();
		}

	}


}
