package frc.robot.subsystems.elevator.factory;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.motors.TalonFXWrapper;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.records.ElevatorMotorStuff;
import frc.robot.subsystems.elevator.records.ElevatorRequests;
import frc.robot.subsystems.elevator.records.ElevatorSignals;
import frc.utils.AngleUnit;

public class RealElevatorConstants {

    private static int LIMIT_SWITCH_CHANNEL = 0;
    private static double LIMIT_SWITCH_DEBOUNCE_TIME = 0.04;

    public static Elevator generate(String logPath){
        TalonFXMotor firstMotor = new TalonFXMotor(logPath + "FirstMotor/", new Phoenix6DeviceID(1), new SysIdRoutine.Config());
        ElevatorRequests firstMotorRequests = new ElevatorRequests(
                Phoenix6RequestBuilder.build(new PositionVoltage(0)),
                Phoenix6RequestBuilder.build(new VoltageOut(0))
        );
        ElevatorSignals firstMotorSignals = new ElevatorSignals(
                Phoenix6SignalBuilder.generatePhoenix6Signal(firstMotor.getDevice().getPosition(), 60, AngleUnit.ROTATIONS),
                Phoenix6SignalBuilder.generatePhoenix6Signal(firstMotor.getDevice().getMotorVoltage(), 60)
        );
        ElevatorMotorStuff firstMotorStuff = new ElevatorMotorStuff(
                firstMotor,
                firstMotorRequests,
                firstMotorSignals
        );

        TalonFXMotor secondMotor = new TalonFXMotor(logPath + "SecondMotor/", new Phoenix6DeviceID(1), new SysIdRoutine.Config());
        ElevatorRequests secondMotorRequests = new ElevatorRequests(
                Phoenix6RequestBuilder.build(new PositionVoltage(0)),
                Phoenix6RequestBuilder.build(new VoltageOut(0))
        );
        ElevatorSignals secondMotorSignals = new ElevatorSignals(
                Phoenix6SignalBuilder.generatePhoenix6Signal(secondMotor.getDevice().getPosition(), 60, AngleUnit.ROTATIONS),
                Phoenix6SignalBuilder.generatePhoenix6Signal(secondMotor.getDevice().getMotorVoltage(), 60)
        );
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
