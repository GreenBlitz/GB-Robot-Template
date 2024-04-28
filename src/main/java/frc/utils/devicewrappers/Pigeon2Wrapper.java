package frc.utils.devicewrappers;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Phoenix6Constants;

public class Pigeon2Wrapper extends Pigeon2 {

    private double rollOffSetDeg;

    private double pitchOffSetDeg;

    public Pigeon2Wrapper(int id) {
        this(id, Phoenix6Constants.CANBUS_NAME, new Pigeon2Configuration());
    }

    public Pigeon2Wrapper(int id, String busChain) {
        this(id, busChain, new Pigeon2Configuration());
    }

    public Pigeon2Wrapper(int id, Pigeon2Configuration configuration) {
        this(id, Phoenix6Constants.CANBUS_NAME, configuration);
    }

    public Pigeon2Wrapper(int id, String busChain, Pigeon2Configuration configuration) {
        super(id, busChain);
        this.rollOffSetDeg = 0;
        this.pitchOffSetDeg = 0;
        applyConfiguration(configuration);
    }

    public void applyConfiguration(Pigeon2Configuration configuration) {
        getConfigurator().apply(configuration);
    }

    public StatusCode setYaw(Rotation2d newYaw) {
        return super.setYaw(newYaw.getDegrees());
    }

    /**
     * This function only set pitch for the function "getAdjustedRoll".
     * If you want to use the pitch signal you need to use "getRoll" but note to yourself that it won't be affected by "setRoll".
     * If for some reason you want to get the needed offset to make the returned value affected by resets use "getRollOffset".
     *
     * @param newRoll - the wanted roll of the gyro to have
     */
    public void setRoll(Rotation2d newRoll) {
        rollOffSetDeg = newRoll.getDegrees() - getRoll().getValue();
    }

    /**
     * This function only set pitch for the function "getAdjustedPitch".
     * If you want to use the pitch signal you need to use "getPitch" but note to yourself that it won't be affected by
     * "setPitch".
     * If for some reason you want to get the needed offset to make the returned value affected by resets use "getPitchOffset".
     *
     * @param newPitch - the wanted pitch of the gyro to have
     */
    public void setPitch(Rotation2d newPitch) {
        pitchOffSetDeg = newPitch.getDegrees() - getPitch().getValue();
    }

    public Rotation2d getRollOffSet() {
        return Rotation2d.fromDegrees(rollOffSetDeg);
    }

    public Rotation2d getPitchOffSet() {
        return Rotation2d.fromDegrees(pitchOffSetDeg);
    }

    public Rotation2d getAdjustedRoll() {
        return Rotation2d.fromDegrees(getRoll().getValue() + rollOffSetDeg);
    }

    public Rotation2d getAdjustedPitch() {
        return Rotation2d.fromDegrees(getPitch().getValue() + pitchOffSetDeg);
    }

}
