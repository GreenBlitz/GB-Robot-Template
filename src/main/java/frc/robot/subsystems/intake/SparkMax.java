package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxMotor;
import frc.robot.hardware.signal.cansparkmax.SparkMaxDoubleSignal;

import javax.print.DocFlavor;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkMax {

    private final static double DEBOUNCE_TIME_SECONDS = 7;//find

    private static final String SIGNAL_NAME = "Yosi";//do

    private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kRising;

    public static BrushlessSparkMAXMotor getMotor() {
        CANSparkMax motor = new CANSparkMax(IDs.CANSparkMaxIDs.INTAKE_ID.ID(), IDs.CANSparkMaxIDs.INTAKE_ID.type());
        BrushlessSparkMAXMotor
    }

    public static SuppliedDigitalInput getBeamBreaker() {
        BooleanSupplier isPressed = () -> getMotor().getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
        SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isPressed, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);//maybe forward
        return beamBreaker;
    }

    public static SparkMaxDoubleSignal getSignal(){
        Supplier<Double> appliedOutput = ()-> getMotor().getAppliedOutput();
        SparkMaxDoubleSignal signal = new SparkMaxDoubleSignal(SIGNAL_NAME, appliedOutput);
        return signal;
    }

    public IntakeStuff generateIntakeStuff(){
        return new IntakeStuff(getMotor(), getSignal(), getBeamBreaker())
    }

}
