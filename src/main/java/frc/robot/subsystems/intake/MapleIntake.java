package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.GBSubsystem;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

public class MapleIntake extends GBSubsystem {

	private final IntakeSimulation intakeSimulation;
	private boolean running;

	public MapleIntake(String logPath, IntakeSimulation intakeSimulation) {
		super(logPath);
		this.intakeSimulation = intakeSimulation;
	}

	@Override
	protected void subsystemPeriodic() {
		Logger.recordOutput(getLogPath() + "/IsNoteIn", isNoteInsideIntake());
		Logger.recordOutput(getLogPath() + "/IsRunning", running);
	}

	public void setRunning(boolean runIntake) {
		running = runIntake;
		if (running)
			intakeSimulation.startIntake();
		else
			intakeSimulation.stopIntake();
	}

	public boolean isNoteInsideIntake() {
		return intakeSimulation.getGamePiecesAmount() != 0;
	}

	public boolean releaseNote() {
		return intakeSimulation.obtainGamePieceFromIntake();
	}

	private static final Translation2d shooterTranslationOnRobot = new Translation2d(0.1, 0);
	private static final double shooterHeightMeters = 0.45, shooterPitchAngleRad = -Math.toRadians(55);

	public void visualizeNoteInIntake(Pose2d robotPose) {
		Translation2d note2dCenter = robotPose.getTranslation().plus(shooterTranslationOnRobot.rotateBy(robotPose.getRotation()));
		Translation3d notePosition = new Translation3d(note2dCenter.getX(), note2dCenter.getY(), shooterHeightMeters);
		Rotation3d noteRotation = new Rotation3d(0, shooterPitchAngleRad, robotPose.getRotation().getRadians());
		Logger.recordOutput("Intake/NoteInIntake", isNoteInsideIntake() ? new Pose3d[] {new Pose3d(notePosition, noteRotation)} : new Pose3d[0]);
	}

}
