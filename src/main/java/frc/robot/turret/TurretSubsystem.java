package frc.robot.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.GBSubsystem;

public class TurretSubsystem extends GBSubsystem {

    ITurret turret;
    TurretState state;
    TurretInputsAutoLogged inputs;

    public TurretSubsystem(ITurret turret) {
        this.turret = turret;
        this.state = TurretState.HOLD_POSITION_RELATIVE_TO_ROBOT;
        inputs = new TurretInputsAutoLogged();
    }


    public void setState(TurretState targetState) {
        this.state = targetState;
    }

    public Rotation2d getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
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
        this.turret.setPosition(targetAngle);
    }

    public void handleHoldPositionRelativeToRobot() {
        this.turret.setPosition(getPosition());
    }

    public void handleRest() {
        this.turret.stop();
    }

    private void handleState(TurretState state) {
        switch (state) {
            case ROTATE_TO_POINT -> handleRotateToPoint(
                    TurretConstants.LOOKING_TARGET,
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
        handleState(this.state);
        turret.updateInputs(inputs);
    }

}
