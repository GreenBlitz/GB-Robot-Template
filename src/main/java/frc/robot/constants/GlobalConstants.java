package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;

public class GlobalConstants {

	public static final double DEFAULT_SIGNALS_FREQUENCY_HERTZ = 60;

	public static final DriverStation.Alliance SIMULATION_ALLIANCE = DriverStation.Alliance.Blue;

	public static final CANBus CAN_BUS = new CANBus();

}
