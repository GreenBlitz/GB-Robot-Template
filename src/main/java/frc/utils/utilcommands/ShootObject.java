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
        timer = new Timer();
    }

    public ShootObject(double durationSeconds, double speedMetersPerSecond, Supplier<Rotation2d> shooterAngle,
                       Translation3d shooterPositionRelativeToRobot,Supplier<Pose3d> swervePosition) {
        this(durationSeconds, speedMetersPerSecond, shooterAngle, shooterPositionRelativeToRobot, swervePosition, 0);
    }

    @Override
    public void initialize() {
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

    private Translation3d getPosition(double time) {

        double relativePosition = time * speed;

        return startingNotePosition.
                times(relativePosition).
                plus(shooterPose3D.getTranslation()).
                minus(
                        new Translation3d(
                            0,
                            0,
                            (gravity / 2 * Math.pow(time,2))
                        )
                );
    }

    @Override
    public void execute() {

        Logger.recordOutput(
                "ObjectVisualizer",
                new Pose3d[]{
                        new Pose3d(
                                getPosition(timer.get()),
                                shooterRotation.unaryMinus()
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
        timer.stop();
        timer.reset();
        Logger.recordOutput("ObjectVisualizer", new Pose3d[]{});
    }
}
