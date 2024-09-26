package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;

import javax.print.DocFlavor;

public class Constants {

    private static final String LOG_PATH = "";

    private static final int MOTOR_ID = 0;

    private static int DIGITAL_INPUT_CHANNEL = 0;

    public static IntakeStuff sparkMax() {
        CANSparkMax motor = new CANSparkMax(MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    }

    public static IntakeStuff talonFX() {
        TalonFXWrapper talonFXWrapper = new TalonFXWrapper(MOTOR_ID);
        SysIdRoutine.Config config = new SysIdRoutine.Config();//enter stuff
        TalonFXMotor motor = new TalonFXMotor(LOG_PATH, talonFXWrapper, config);

    }

    public static IDigitalInput beamBreaker(){
        DigitalInput beamBreaker = new DigitalInput(DIGITAL_INPUT_CHANNEL);
        return beamBreaker;
    }

}
