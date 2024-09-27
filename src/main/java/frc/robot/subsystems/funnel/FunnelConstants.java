package frc.robot.subsystems.funnel;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;

public class FunnelConstants {

    public static final String LOG_PATH = "Subsystems/funnel/";

    public static final boolean ENABLE_FORWARD_LIMIT_SWITCH = false;

}
