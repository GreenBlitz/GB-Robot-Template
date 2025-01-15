package frc.robot.subsystems.elevator.factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.records.ElevatorMotorStuff;
import frc.robot.subsystems.elevator.records.ElevatorRequests;
import frc.robot.subsystems.elevator.records.ElevatorSignals;
import frc.utils.AngleUnit;

public class RealElevatorConstants {

    private static final int LIMIT_SWITCH_CHANNEL = 0;
    private static final double LIMIT_SWITCH_DEBOUNCE_TIME = 0.04;
    private static final SysIdRoutine.Config FIRST_MOTOR_CONFIG = new SysIdRoutine.Config();
    private static final SysIdRoutine.Config SECOND_MOTOR_CONFIG = new SysIdRoutine.Config();

    private static void configMotor(TalonFXMotor motor){
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Slot0.withKP(1).withKI(1).withKD(1);
        configuration.CurrentLimits.StatorCurrentLimit = 40;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS);
    }

    private static ElevatorRequests createRequests(){
        return new ElevatorRequests(
                Phoenix6RequestBuilder.build(new PositionVoltage(0).withEnableFOC(true)),
                Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true))
        );
    }

    private static ElevatorSignals createSignals(TalonFXMotor motor){
        return new ElevatorSignals(
                Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS),
                Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ)
        );
    }

    public static Elevator generate(String logPath){
        TalonFXMotor firstMotor = new TalonFXMotor(logPath + "FirstMotor/", IDs.Phoenix6IDs.ELEVATOR_FIRST_MOTOR_ID, FIRST_MOTOR_CONFIG);
        configMotor(firstMotor);
        ElevatorRequests firstMotorRequests = createRequests();
        ElevatorSignals firstMotorSignals = createSignals(firstMotor);
        );
        ElevatorMotorStuff firstMotorStuff = new ElevatorMotorStuff(
                firstMotor,
                firstMotorRequests,
                firstMotorSignals
        );

        TalonFXMotor secondMotor = new TalonFXMotor(logPath + "SecondMotor/", IDs.Phoenix6IDs.ELEVATOR_SECOND_MOTOR_ID, SECOND_MOTOR_CONFIG);
        configMotor(secondMotor);
        ElevatorRequests secondMotorRequests = createRequests();
        ElevatorSignals secondMotorSignals = createSignals(secondMotor);
        ElevatorMotorStuff secondMotorStuff = new ElevatorMotorStuff(
                secondMotor,
                secondMotorRequests,
                secondMotorSignals
        );

        ChanneledDigitalInput limitSwitch = new ChanneledDigitalInput(new DigitalInput(LIMIT_SWITCH_CHANNEL), new Debouncer(LIMIT_SWITCH_DEBOUNCE_TIME));

        return new Elevator(
                logPath,
                logPath + "LimitSwitch/",
                firstMotorStuff,
                secondMotorStuff,
                limitSwitch
        );
    }

}
