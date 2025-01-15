package frc.robot.subsystems.elevator.factory;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.mechanisms.wpilib.ElevatorSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.records.ElevatorMotorStuff;
import frc.robot.subsystems.elevator.records.ElevatorRequests;
import frc.robot.subsystems.elevator.records.ElevatorSignals;
import frc.utils.AngleUnit;

public class SimulationElevatorConstants {

    public static Elevator generate(String logPath){
        ElevatorSimulation elevatorSimulation = new ElevatorSimulation(
                new ElevatorSim(
                        LinearSystemId.createElevatorSystem(
                                DCMotor.getKrakenX60Foc(2),
                                10,
                                0.025,
                                ElevatorConstants.DRUM_RADIUS
                        ),
                        DCMotor.getKrakenX60Foc(2),
                        0,
                        2,
                        false,
                        0
                ),
                ElevatorConstants.DRUM_RADIUS,
                (double) 1 / 5
        );
        TalonFXMotor firstMotor = new TalonFXMotor(logPath + "FirstMotor/", new Phoenix6DeviceID(1), new SysIdRoutine.Config(), elevatorSimulation);
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

        SuppliedDigitalInput digitalInput = new SuppliedDigitalInput(() -> false, new Debouncer(0.04));

        return new Elevator(
                logPath,
                logPath + "LimitSwitch/",
                firstMotorStuff,
                firstMotorStuff,
                digitalInput
        );
    }

}
