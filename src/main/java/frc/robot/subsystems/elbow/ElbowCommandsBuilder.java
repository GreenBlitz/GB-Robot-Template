package frc.robot.subsystems.elbow;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class ElbowCommandsBuilder {

    private final Elbow elbow;

    public ElbowCommandsBuilder(Elbow elbow) {
        this.elbow = elbow;
    }

    public Command moveToAngle(Rotation2d angle, Rotation2d tolerance) {
        return new FunctionalCommand(
                () -> elbow.setTargetAngle(angle),
                () -> {},
                interrupted -> elbow.stayInPlace(),
                () -> elbow.isAtAngle(angle, tolerance)
        ).withName("Move to Angle: " + angle);
    }

}
