package frc.robot.simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSimulation {

    private double yawRadians = 0;

    private double rollRadians = 0;

    private double pitchRadians = 0;

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromRadians(yawRadians);
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromRadians(rollRadians);
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromRadians(pitchRadians);
    }

    public void updateYaw(Rotation2d omegaPerSecond, double timeSeconds) {
        yawRadians += omegaPerSecond.getRadians() * timeSeconds;
    }

    public void updateRoll(Rotation2d omegaPerSecond, double timeSeconds) {
        rollRadians += omegaPerSecond.getRadians() * timeSeconds;
    }

    public void updatePitch(Rotation2d omegaPerSecond, double timeSeconds) {
        pitchRadians += omegaPerSecond.getRadians() * timeSeconds;
    }

    public void setYaw(Rotation2d yaw) {
        yawRadians = yaw.getRadians();
    }

    public void setRoll(Rotation2d roll) {
        rollRadians = roll.getRadians();
    }

    public void setPitch(Rotation2d pitch) {
        pitchRadians = pitch.getRadians();
    }

    public Rotation2d getRangedYaw() {
        return rangeAngle(Rotation2d.fromRadians(yawRadians));
    }

    public Rotation2d getRangedRoll() {
        return rangeAngle(Rotation2d.fromRadians(rollRadians));
    }

    public Rotation2d getRangedPitch() {
        return rangeAngle(Rotation2d.fromRadians(pitchRadians));
    }

    /**
     * Range angle between -180, 180
     *
     * @param angle - the angle to range
     * @return the ranged angle
     */
    private Rotation2d rangeAngle(Rotation2d angle) {
        return Rotation2d.fromRadians(MathUtil.angleModulus(angle.getRadians()));
    }

}
