package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;

public class Constants {

    protected final String LOG_PATH = "";

    private final int MOTOR_ID = 0;

    protected CANSparkMax sparkMax() {
        CANSparkMax motor = new CANSparkMax(MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
//        InputSignal<Double> voltage = new SparkMaxAngleSignal
        return motor;
    }

    protected TalonFX talonFX() {
        TalonFX motor = new TalonFX(MOTOR_ID, ""); //CANBUS
//        InputSignal<Double> voltage = new
        return motor;
    }

    protected Intake getIntake(){
        Intake intake = new
    }


}
