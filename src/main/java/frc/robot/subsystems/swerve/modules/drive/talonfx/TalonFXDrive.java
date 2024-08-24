package frc.robot.subsystems.swerve.modules.drive.talonfx;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.talonfx.TalonFXMotor;
import frc.robot.subsystems.swerve.modules.drive.DriveThreadMetersInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.phoenix6signalsthread.Phoenix6SignalsThread;

import java.util.Queue;

public class TalonFXDrive extends TalonFXMotor implements IDrive {

	private final Queue<Double> drivePositionQueue;

	public TalonFXDrive(TalonFXDriveConstants constants) {
		super(constants.getMotor(), constants.getSignals(), constants.getSysidConfig());
		this.drivePositionQueue = Phoenix6SignalsThread.getInstance().registerLatencySignal(motor, signals.position(), signals.velocity());
	}

	@Override
	public void updateInputs(DriveThreadMetersInputsAutoLogged driveThreadMetersInputs) {
		driveThreadMetersInputs.angleOdometrySamples = drivePositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
		drivePositionQueue.clear();
	}

}
