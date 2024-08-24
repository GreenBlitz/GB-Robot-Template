package frc.robot.subsystems.swerve.modules.steer.talonfx;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.talonfx.TalonFXMotor;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.SteerThreadInputsAutoLogged;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;

import java.util.Queue;

public class TalonFXSteer extends TalonFXMotor implements ISteer {

	private final Queue<Double> positionQueue;

	public TalonFXSteer(TalonFXSteerConstants constants) {
		super(constants.getMotor(), constants.getSignals(), constants.getSysidConfig());
		this.positionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(motor, signals.position(), signals.velocity());
	}

	@Override
	public void updateInputs(SteerThreadInputsAutoLogged steerThreadInputs) {
		steerThreadInputs.angleOdometrySamples = positionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
		positionQueue.clear();
	}

}
