package frc.robot.hardware.phoenix6.imu;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.math.util.MathUtil;
import frc.utils.costomUnits.Angle2d;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.Phoenix6Util;

import static org.wpilib.units.Units.Degrees;

public class Pigeon2Wrapper extends Pigeon2 {

	//@formatter:off
    /*THANKS SO MUCH TO TEAM 4481
    Hier woont een duif
                     ░░░░
              ░░░░░░░░▒▒░░░░░░
          ░░░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░
        ░░░░▒▒▒▒▒▒▒▒▒▒░░░░▒▒▒▒▒▒░░
      ░░░░▒▒▒▒▒▒▒▒▒▒░░▒▒▒▒░░▒▒▒▒▒▒
      ░░░░▒▒▒▒▒▒▒▒▒▒░░  ▓▓▒▒░░▒▒▒▒▒▒
      ▒▒░░░░▒▒▒▒▒▒▒▒░░▓▓▓▓▓▓░░▒▒▒▒▒▒▒▒
    ░░░░░░░░░░░░▒▒▒▒░░▒▒▓▓▓▓░░▒▒▒▒▒▒▒▒
  ░░░░▒▒▓▓░░░░░░▒▒▒▒▒▒░░░░░░▒▒▒▒▒▒▒▒▒▒▒▒
  ░░▒▒▒▒▓▓▒▒░░░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  ▒▒░░▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░
  ░░░░▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  ▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
░░░░▒▒▓▓▓▓▓▓████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░
░░▒▒▓▓████████▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
  ▒▒          ▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
              ▒▒▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ░░▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
                ▓▓▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
                ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
                ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░
                ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ░░▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ░░▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ▒▒▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░▒▒▒▒▒▒▓▓▓▓▓▓▓▓░░
              ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒░░░░░░░░
            ▒▒▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▒▒▒▒▓▓▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░
            ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▓▓▓▓▒▒░░░░░░░░░░░░░░░░░░░░░░░░▒▒░░
          ▒▒▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▒▒▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▒▒░░
          ▓▓▓▓▓▓▒▒▒▒▒▒▓▓▒▒▓▓▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▓▓▓▓▓▓▒▒░░                                                        ░░
        ░░▓▓▓▓▒▒▓▓▒▒▓▓▓▓▒▒▓▓▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▓▓▓▓▓▓▓▓░░░░░░░░░░                                      ▒▒██▓▓▓▓▓▓
        ▓▓▓▓▓▓▒▒▓▓▒▒▓▓▓▓▓▓▓▓▒▒▓▓▒▒▓▓▒▒▒▒▒▒▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▒▒▓▓▓▓░░░░░░░░░░░░▓▓▒▒░░░░                  ░░▓▓▓▓▓▓▓▓▓▓▓▓▓▓
        ▓▓▓▓▒▒▓▓▓▓▒▒▓▓▓▓▓▓▓▓▒▒▓▓▒▒▓▓▒▒▓▓▒▒▓▓▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░    ░░░░░░▒▒▒▒▓▓▓▓▓▓▒▒░░░░░░░░░░░░▒▒▓▓▓▓░░░░░░░░░░░░  ▒▒▓▓▓▓▓▓▓▓▓▓▓▓▒▒▓▓▒▒
        ▓▓▓▓▒▒▓▓▓▓▓▓▓▓▒▒▓▓▓▓▓▓▒▒▒▒▓▓▒▒▒▒▓▓▓▓▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓░░░░░░░░  ░░░░  ░░▓▓▒▒░░░░░░░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓▒▒▓▓██░░
      ░░▓▓▒▒▒▒▓▓▒▒▓▓▓▓▒▒▓▓▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░░░  ░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▒▒░░░░░░░░░░░░░░▒▒▓▓░░░░░░░░░░░░▒▒▓▓▓▓▓▓▓▓▒▒▓▓▓▓▓▓
      ░░▓▓▒▒▒▒▓▓▒▒▓▓▒▒▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  ▓▓▒▒░░░░░░░░░░░░▒▒▓▓▓▓▒▒▒▒░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
        ░░▒▒▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓░░░░░░░░░░░░░░▒▒▓▓▓▓▓▓░░▒▒██▓▓▓▓▓▓▓▓▓▓██▓▓████▓▓▓▓▒▒
          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓░░░░░░░░░░▒▒▓▓▓▓░░░░░░▓▓▓▓▓▓████▓▓▓▓▓▓▓▓▓▓▒▒
          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▓▓▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  ░░▓▓▓▓▓▓▓▓▓▓░░░░░░░░▒▒▓▓▓▓░░░░░░▓▓▓▓██▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒
          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▒▒▒▒░░░░░░░░░░░░▓▓▓▓▓▓▓▓▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒
          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  ░░░░░░░░  ░░░░░░░░░░░░░░░░▒▒▓▓▓▓▓▓▓▓▓▓░░░░░░░░░░░░░░▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒
            ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒░░░░▒▒░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓██▓▓▓▓▓▓▓▓██▒▒
            ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▓▓▒▒▒▒▒▒▓▓▓▓░░  ░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░▓▓▓▓▓▓▓▓░░░░░░░░░░░░▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒
            ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▒▒░░░░▒▒░░░░░░░░░░░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓▒▒░░░░░░░░▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░
              ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▒▒░░░░░░▒▒░░░░░░░░░░░░░░▓▓▓▓▒▒░░░░░░▒▒▓▓████▓▓▓▓████▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒░░
              ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓░░░░░░▓▓▓▓▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓▓▓████▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░
                ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒░░░░▓▓▓▓▓▓▒▒▒▒░░░░░░░░░░░░░░░░▒▒██▓▓▓▓▓▓▓▓▓▓██▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▓▓▒▒
                  ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒░░░░░░░░░░░░▒▒▒▒▓▓██▓▓██▓▓▓▓▓▓▓▓▓▓▒▒▓▓██▓▓██▓▓▓▓▒▒░░░░
                    ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▓▓██
                      ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  ░░▒▒▓▓▓▓▓▓▒▒▓▓▓▓▓▓▓▓██
                          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▓▓▓▓▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░              ░░░░▒▒
                              ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒
                                  ▒▒▓▓▒▒▒▒▒▒▒▒▒▒▓▓▒▒▓▓▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒
                                        ░░▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
                                                      ▒▒▒▒▒▒▒▒▒▒▒▒      ▒▒▒▒▒▒▒▒▒▒▒▒
                                                      ░░▒▒▒▒▒▒▒▒▒▒      ░░▒▒░░▒▒▒▒▒▒
                                                        ░░▒▒▒▒▒▒▒▒        ░░░░▒▒▒▒▒▒
                                                        ░░▒▒▒▒▒▒          ░░░░▒▒▒▒▒▒
                                                          ▒▒▒▒▒▒            ░░▒▒▒▒▒▒
                                                          ▒▒▓▓              ▒▒▒▒▒▒░░
                                                          ▒▒░░              ▒▒▒▒▒▒
                                                        ░░▒▒                ▒▒▒▒
                                                      ░░▒▒                  ▒▒░░
                                        ▒▒▒▒░░░░░░░░▒▒▒▒▒▒▒▒▓▓▓▓          ▒▒▒▒
                                      ▒▒▒▒▒▒▒▒▒▒░░▒▒▒▒▒▒▒▒▒▒░░            ░░▒▒░░
                                          ░░░░▒▒▒▒▒▒░░▒▒                ▒▒▒▒▒▒▒▒▒▒██▒▒
                                        ▒▒▒▒▒▒░░  ░░▒▒░░      ░░▒▒░░▒▒▒▒░░▒▒▒▒▒▒░░
                                      ▓▓▓▓░░      ░░▒▒░░  ░░▒▒▒▒▒▒▒▒░░▒▒▒▒░░▒▒▒▒
                                      ░░          ▒▒▓▓    ▓▓    ░░▒▒▒▒░░░░▒▒░░▒▒
                                                  ░░▒▒        ▒▒▒▒          ▒▒▒▒░░
                                                  ░░        ██▒▒              ▒▒▒▒
                                                            ▒▒

     */
    //@formatter:on

	private static final int DEFAULT_CONFIG_NUMBER_OF_TRIES = 1;

	private final BusChain busChain;

	private double rollOffSetDegrees;

	private double pitchOffSetDegrees;

	public Pigeon2Wrapper(int id) {
		this(new Phoenix6DeviceID(id));
	}

	public Pigeon2Wrapper(Phoenix6DeviceID deviceID) {
		super(deviceID.id(), deviceID.busChain().getCANBus());
		this.busChain = deviceID.busChain();
		this.rollOffSetDegrees = 0;
		this.pitchOffSetDegrees = 0;
	}

	public StatusCode applyConfiguration(Pigeon2Configuration configuration, int numberOfTries) {
		return Phoenix6Util.checkStatusCodeWithRetry(() -> getConfigurator().apply(configuration), numberOfTries);
	}

	public StatusCode applyConfiguration(Pigeon2Configuration configuration) {
		return applyConfiguration(configuration, DEFAULT_CONFIG_NUMBER_OF_TRIES);
	}

	public StatusCode setYaw(Angle2d newYaw) {
		return super.setYaw(newYaw.getDegrees());
	}

	public Angle2d getAngle2dYaw() {
		return Angle2d.fromDegrees(getYaw().getValue().in(Degrees));
	}

	/**
	 * This function only set pitch for the function "getAdjustedRoll". If you want to use the pitch signal you need to use "getRoll" but note to
	 * yourself that it won't be affected by "setRoll". If for some reason you want to get the needed offset to make the returned value affected
	 * by resets use "getRollOffset".
	 *
	 * @param newRoll - the wanted roll of the IMU to have
	 */
	public void setRoll(Angle2d newRoll) {
		rollOffSetDegrees = calculateOffset(newRoll.getDegrees(), getRoll().getValue().in(Degrees));
	}

	/**
	 * This function only set pitch for the function "getAdjustedPitch". If you want to use the pitch signal you need to use "getPitch" but note
	 * to yourself that it won't be affected by "setPitch". If for some reason you want to get the needed offset to make the returned value
	 * affected by resets use "getPitchOffset".
	 *
	 * @param newPitch - the wanted pitch of the IMU to have
	 */
	public void setPitch(Angle2d newPitch) {
		pitchOffSetDegrees = calculateOffset(newPitch.getDegrees(), getPitch().getValue().in(Degrees));
	}

	public Angle2d getRollOffSet() {
		return Angle2d.fromDegrees(rollOffSetDegrees);
	}

	public Angle2d getPitchOffSet() {
		return Angle2d.fromDegrees(pitchOffSetDegrees);
	}

	public Angle2d getAdjustedRoll() {
		return Angle2d.fromDegrees(getRoll().getValue().in(Degrees) + rollOffSetDegrees);
	}

	public Angle2d getAdjustedPitch() {
		return Angle2d.fromDegrees(getPitch().getValue().in(Degrees) + pitchOffSetDegrees);
	}

	/**
	 * @return - the yaw value ranged between -180 , 180
	 */
	public Angle2d getRangedYaw() {
		return rangeAngle(Angle2d.fromDegrees(getYaw().getValue().in(Degrees)));
	}

	/**
	 * @return - the unadjusted roll value ranged between -180 , 180
	 */
	public Angle2d getRangedRoll() {
		return rangeAngle(Angle2d.fromDegrees(getRoll().getValue().in(Degrees)));
	}

	/**
	 * @return - the unadjusted pitch value ranged between -180 , 180
	 */
	public Angle2d getRangedPitch() {
		return rangeAngle(Angle2d.fromDegrees(getPitch().getValue().in(Degrees)));
	}

	/**
	 * @return - the adjusted roll value ranged between -180 , 180
	 */
	public Angle2d getAdjustedRangedRoll() {
		return rangeAngle(getAdjustedRoll());
	}

	/**
	 * @return - the adjusted pitch value ranged between -180 , 180
	 */
	public Angle2d getAdjustedRangedPitch() {
		return rangeAngle(getAdjustedPitch());
	}

	/**
	 * @param newAngleDegrees     - the wanted angle in degrees
	 * @param currentAngleDegrees - the current angle in degrees
	 * @return the offset
	 */
	private double calculateOffset(double newAngleDegrees, double currentAngleDegrees) {
		return newAngleDegrees - currentAngleDegrees;
	}

	/**
	 * Range angle between -180, 180
	 *
	 * @param angle - the angle to range
	 * @return the ranged angle
	 */
	private Angle2d rangeAngle(Angle2d angle) {
		return Angle2d.fromRadians(MathUtil.angleModulus(angle.getRadians()));
	}

	public BusChain getBusChain() {
		return busChain;
	}

}
