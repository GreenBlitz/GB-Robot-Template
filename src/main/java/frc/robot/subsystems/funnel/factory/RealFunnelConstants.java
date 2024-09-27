package frc.robot.subsystems.funnel.factory;

import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.cansparkmax.SparkMaxDoubleSignal;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.FunnelStuff;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RealFunnelConstants {

    private final static double DEBOUNCE_TIME_SECONDS = 0.05;

    private static final String SIGNAL_NAME = "voltage";

    private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kRising;// idk

    public static FunnelStuff generateFunnelStuff(String logPath) {
        SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMaxes.FUNNEL);
        SysIdRoutine.Config config = new SysIdRoutine.Config();// config!
        BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, config);

        Supplier<Double> voltage = () -> (sparkMaxWrapper.getBusVoltage() * sparkMaxWrapper.getAppliedOutput());
        SparkMaxDoubleSignal signal = new SparkMaxDoubleSignal(SIGNAL_NAME, voltage);

        BooleanSupplier isPressed = () -> sparkMaxWrapper.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
        sparkMaxWrapper.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
                .enableLimitSwitch(FunnelConstants.ENABLE_FORWARD_LIMIT_SWITCH);

        SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isPressed, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);// maybe forward

        return new FunnelStuff(logPath, motor, signal, beamBreaker);
    }

}
