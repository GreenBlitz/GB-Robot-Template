package frc.robot.subsystems.swerve.modules.drive.talonfx;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.talonfx.TalonFXMotor;
import frc.robot.subsystems.swerve.modules.drive.DriveThreadMetersInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;

import java.util.Queue;

public class TalonFXDrive extends TalonFXMotor implements IDrive {

	private final Queue<Double> drivePositionQueue;

	public TalonFXDrive(TalonFXDriveConstants constants) {
		super(constants.getMotor(), constants.getSignals(), constants.getSysidConfig());

		this.drivePositionQueue = PhoenixOdometryThread6328.getInstance()
			.registerLatencySignal(mMotor, mSignals.position(), mSignals.velocity());
	}

	@Override
	public void updateInputs(DriveThreadMetersInputsAutoLogged driveThreadMetersInputs) {
		driveThreadMetersInputs.angleOdometrySamples = drivePositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
		drivePositionQueue.clear();
	}

}
