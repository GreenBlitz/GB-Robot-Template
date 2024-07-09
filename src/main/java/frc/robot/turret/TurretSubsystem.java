package frc.robot.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Objects;

public class TurretSubsystem extends GBSubsystem {

    ITurret turret;
    TurretState state;
    TurretInputsAutoLogged inputs;

    public TurretSubsystem(ITurret turret) {
        this.turret = turret;
        inputs = new TurretInputsAutoLogged();
        turret.updateInputs(inputs);

        this.state = TurretState.REST;
    }


    public void setState(TurretState targetState) {
        this.state = targetState;
    }

    public Rotation2d getPosition() {
        return inputs.position;
    }

    public Rotation2d getVelocity() {
        return inputs.velocity;
    }


    public void handleRotateToPoint(Translation2d targetPoint, Translation2d robotPosition) {
        Translation2d normalizedRobotPosition = targetPoint.minus(robotPosition);
        this.turret.setPosition(
                new Rotation2d(
                        targetPoint.minus(robotPosition).getX(),
                        targetPoint.minus(robotPosition).getY()
                )
        );
    }

    public void handleHoldPositionRelativeToRobot(Rotation2d targetAngle) {
        this.turret.setPosition(Objects.requireNonNullElseGet(targetAngle, () -> Rotation2d.fromDegrees(0)));
    }

    public void handleHoldPositionRelativeToRobot() {
       handleHoldPositionRelativeToRobot(inputs.position);
    }

    public void handleRest() {
        this.turret.stop();
    }

    private void handleState(TurretState state) {
        switch (state) {
            case ROTATE_TO_POINT -> handleRotateToPoint(
                    new Translation2d(),
                    new Translation2d() /*waiting for swerve to do get from pose estimation*/
            );
            case HOLD_POSITION_RELATIVE_TO_ROBOT -> handleHoldPositionRelativeToRobot();
            case REST -> handleRest();
        }
    }


    @Override
    protected String getLogPath() {
        return null;
    }

    @Override
    protected void subsystemPeriodic() {
//        handleState(this.state);

//        setState(TurretState.ROTATE_TO_POINT);
        turret.setPosition(Rotation2d.fromRotations(0.5));

        turret.updateInputs(inputs);
        Logger.processInputs("Turret",inputs);
    }

}
