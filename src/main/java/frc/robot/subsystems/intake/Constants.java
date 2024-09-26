package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;

public class Constants {

    protected static final String LOG_PATH = "";

    private static final int MOTOR_ID = 0;

    private static int DIGITAL_INPUT_CHANNEL = 0;

    public static CANSparkMax sparkMax() {
        CANSparkMax motor = new CANSparkMax(MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
//        InputSignal<Double> voltage = new SparkMaxAngleSignal
        return motor;
    }
//
//    public static TalonFX talonFX() {
//        TalonFXMotor motor = new TalonFXMotor(LOG_PATH, , );
//        InputSignal<Double> voltage = new
//        return motor;
//    }

    public static DigitalInput beamBreaker() {
        DigitalInput beamBreaker = new DigitalInput(DIGITAL_INPUT_CHANNEL);
        return beamBreaker;
    }

}
