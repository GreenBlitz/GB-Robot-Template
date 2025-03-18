package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.AngleSignal;
import frc.utils.math.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

public class Phoenix6LatencySignal extends AngleSignal implements SignalGetter {

	private final StatusSignal<?> signal;
	protected final StatusSignal<?> slopeSignal;

	protected Phoenix6LatencySignal(String name, StatusSignal<?> signal, StatusSignal<?> slopeSignal, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.signal = signal;
		this.slopeSignal = slopeSignal;
	}

	@Override
	protected void updateValue(TimedValue<Rotation2d> timedValue) {
		timedValue.setValue(angleUnit.toRotation2d(BaseStatusSignal.getLatencyCompensatedValueAsDouble(signal, slopeSignal)));
		timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
	}

	/**
	 * For using refresh all with more signals...
	 */
	@Override
	public StatusSignal<?> getSignal() {
		return signal;
	}

}
