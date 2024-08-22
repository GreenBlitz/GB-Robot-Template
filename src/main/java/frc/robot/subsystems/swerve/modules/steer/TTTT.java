package frc.robot.subsystems.swerve.modules.steer;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.talonfx.TalonFXMotor;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;

import java.util.Queue;

public class TTTT extends TalonFXMotor {

	private final Queue<Double> positionQueue;

	public TTTT() {
		super();
		this.positionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(mMotor, mSignals.position(), mSignals.velocity());
	}

	public void updateInputs(SteerInputs steerInputs) {
		steerInputs.angleOdometrySamples = positionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
		positionQueue.clear();
	}

}
