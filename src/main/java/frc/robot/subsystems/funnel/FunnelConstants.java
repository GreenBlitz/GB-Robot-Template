package frc.robot.subsystems.funnel;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;

public class FunnelConstants {


    public static IDigitalInput getBeamBreaker(CANSparkMax host) {
        return new SuppliedDigitalInput(
                () -> host.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed(),
                Debouncer.DebounceType.kRising,
                0.5
        );
    }

    public static Funnel getFunnel(String logPath) {
        CANSparkMax motor = new CANSparkMax(IDs.CANSparkMaxes.FUNNEL_ID, CANSparkLowLevel.MotorType.kBrushless);

//        return new Funnel(logPath, getBeamBreaker(motor), new );
    }

}
