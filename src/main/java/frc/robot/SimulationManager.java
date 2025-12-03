package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

public class SimulationManager {

    private final String logPath;
    private final Robot robot;

    public SimulationManager(String logPath, Robot robot) {
        this.logPath = logPath;
        this.robot = robot;
    }

    public void logPoses() {
        logIntakePosition3d();
        logTurretPosition3d();
        logHoodPosition3d();
    }

    private void logIntakePosition3d() {
        Logger.recordOutput(logPath +"/Intake/Position", getIntakePosition3d(robot.getFourBar().getPosition()));
    }

    public Pose3d getIntakePosition3d(Rotation2d pitch) {
        return new Pose3d(new Translation3d(0.0, 0.0, 0.0) ,new Rotation3d(0.0, pitch.getRadians(),0.0));
    }

    public void  logTurretPosition3d() {
        Logger.recordOutput(logPath + "/Turret/Position",getTurretPosition3d(robot.getTurret().getPosition()));
    }

    public Pose3d getTurretPosition3d(Rotation2d yaw) {
        return new Pose3d(new Translation3d(0.0,0.0,0.0), new Rotation3d(0.0, 0.0, yaw.getRadians()));
    }

    public void logHoodPosition3d() {
        Logger.recordOutput(logPath +"/Hood/Position",getHoodPosition3d(robot.getHood().getPosition()) );
    }

    public Pose3d getHoodPosition3d(Rotation2d pitch) {
        return new Pose3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0 ,pitch.getRadians(),robot.getTurret().getPosition().getRadians()));
    }
}
