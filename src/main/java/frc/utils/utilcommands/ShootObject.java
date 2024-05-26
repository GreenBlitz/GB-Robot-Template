package frc.utils.utilcommands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShootObject extends Command {

    private Timer timer;

    private double duration;

    private double speed;

    private Supplier<Rotation2d> shooterAngle;

    private Rotation2d actualShooterAngle;

    private Translation3d shooterPosition;

    private Supplier<Pose3d> swervePosition;

    private double gravity;

    private Pose3d shooterPose3D;

    private Rotation2d chassisAngle;

    public ShootObject(double durationSeconds, double speedMetersPerSecond, Supplier<Rotation2d> shooterAngle,
            Translation3d shooterPositionRelativeToRobot,Supplier<Pose3d> swervePosition,
            double gravityMetersPerSecondSquared) {
        this.duration = durationSeconds;
        this.speed = speedMetersPerSecond;
        this.shooterAngle = shooterAngle;
        this.shooterPosition = shooterPositionRelativeToRobot;
        this.gravity = gravityMetersPerSecondSquared;
        this.swervePosition = swervePosition;
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();


        shooterPose3D = swervePosition.get().plus(
                new Transform3d(shooterPosition, new Rotation3d())
        );
        chassisAngle = shooterPose3D.getRotation().toRotation2d();
        actualShooterAngle = shooterAngle.get();
    }

    @Override
    public void execute() {

        Translation3d notePosition = new Translation3d(
                chassisAngle.getCos() * actualShooterAngle.getCos(),
                chassisAngle.getSin() * actualShooterAngle.getCos(),
                actualShooterAngle.getSin()
        ).times(timer.get() * speed);

        Logger.recordOutput(
                "ObjectVisualizer",
                new Pose3d[]{
                        new Pose3d(
                                notePosition.plus(shooterPose3D.getTranslation()),
                                new Rotation3d(0,actualShooterAngle.getRadians(), 0).plus(shooterPose3D.getRotation())
                        )
                }
        );
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("NoteVisualizer", new Pose3d[]{});
    }
}
