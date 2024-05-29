package frc.utils.utilcommands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public abstract class ShootObject extends Command {

    private Timer timer;

    private double duration;

    private double speed;

    private Supplier<Rotation2d> shooterAngle;

    private Translation3d shooterPosition;

    private Supplier<Pose3d> swervePosition;

    private double gravity;

    private Pose3d shooterPose3D;

    private Translation3d startingNotePosition;

    private Rotation3d shooterRotation;

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
        Rotation2d chassisAngle = shooterPose3D.getRotation().toRotation2d();
        Rotation2d startingShooterAngle = shooterAngle.get();
        startingNotePosition = new Translation3d(
                chassisAngle.getCos() * startingShooterAngle.getCos(),
                chassisAngle.getSin() * startingShooterAngle.getCos(),
                startingShooterAngle.getSin()
        );
        shooterRotation = new Rotation3d(0, startingShooterAngle.getRadians(), 0).plus(shooterPose3D.getRotation());
    }

    private Translation3d getPosition() {
        //based on the formula x = x0 + v0*t + a * t^2 / 2   from physics
        double relativePosition = timer.get() * speed + (gravity / 2 * Math.pow(timer.get(),2));

        return startingNotePosition.
                times(relativePosition).
                plus(shooterPose3D.getTranslation());
    }

    @Override
    public void execute() {

        Logger.recordOutput(
                "ObjectVisualizer",
                new Pose3d[]{
                        new Pose3d(
                                getPosition(),
                                shooterRotation
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
