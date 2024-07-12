package frc.robot.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Objects;

public class Turret extends GBSubsystem {

    private final ITurret turret;
    private TurretState state;
    private final TurretInputsAutoLogged inputs;

    public Turret(ITurret turret) {
        this.turret = turret;

        this.inputs = new TurretInputsAutoLogged();
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

    public void setPower(double power) {
        turret.setPower(power);
    }

    public void handleRotateToTarget() {
        this.turret.setPosition(TurretUtils.calculateAbsoluteTargetAngle(
                new Translation2d() //until swerve gets merged
                ,TurretConstants.LOOKING_TARGET));
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
            case ROTATE_TO_TARGET -> handleRotateToTarget();
            case HOLD_POSITION_RELATIVE_TO_ROBOT -> handleHoldPositionRelativeToRobot();
            case REST -> handleRest();
        }
    }


    @Override
    protected String getLogPath() {
        return TurretConstants.LOGGING_PATH;
    }

    @Override
    protected void subsystemPeriodic() {
        handleState(this.state);

        turret.updateInputs(inputs);
        Logger.processInputs(getLogPath(), inputs);

        Logger.recordOutput(getLogPath() + "state", state);
    }

}
