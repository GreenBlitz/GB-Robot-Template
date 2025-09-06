package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.AngleArraySignal;
import frc.robot.subsystems.swerve.OdometryThread;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;

import java.util.Queue;

public class Phoenix6AngleThreadSignal extends AngleArraySignal {

	private final Queue<TimedValue<Double>> timedValues;

	protected Phoenix6AngleThreadSignal(String name, StatusSignal<?> statusSignal, AngleUnit angleUnit, OdometryThread thread) {
		super(name, angleUnit);
		this.timedValues = thread.addSignal(statusSignal);
	}

	@Override
	protected void updateValues(TimedValue<Rotation2d>[] timedValues) {
		timedValues = new TimedValue[this.timedValues.size()];
		for (int i = 0; i < timedValues.length; i++) {
			TimedValue<Double> value = this.timedValues.poll();
			timedValues[i].setValue(angleUnit.toRotation2d(value.getValue()));
			timedValues[i].setTimestamp(value.getTimestamp());
		}
	}

}
