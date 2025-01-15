package frc.robot.subsystems.elevator.factory;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
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

    private static final double LIMIT_SWITCH_DEBOUNCE_TIME = 0.04;
    private static final int DEFAULT_SIGNALS_FREQUENCY_HERTZ = 60;

    private static final double MASS_KG = 10;
    private static final double RADIUS_METERS = 0.025;
    private static final int NUMBER_OF_MOTORS = 2;
    private static final double MIN_HEIGHT_METERS = 0;
    private static final double MAX_HEIGHT_METERS = 2;
    private static final double STARTING_HEIGHT_METERS = 0;
    private static final double GEAR_RATIO = 1.0 / 5.0;

    public static Elevator generate(String logPath){
        ElevatorSimulation elevatorSimulation = new ElevatorSimulation(
                new ElevatorSim(
                        LinearSystemId.createElevatorSystem(
                                DCMotor.getKrakenX60Foc(2),
                                MASS_KG,
                                RADIUS_METERS,
                                ElevatorConstants.DRUM_RADIUS
                        ),
                        DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS),
                        MIN_HEIGHT_METERS,
                        MAX_HEIGHT_METERS,
                        false,
                        STARTING_HEIGHT_METERS
                ),
                ElevatorConstants.DRUM_RADIUS,
                GEAR_RATIO
        );
        TalonFXMotor firstMotor = new TalonFXMotor(logPath + "FirstMotor/", IDs.ELEVATOR_FIRST_MOTOR_ID, new SysIdRoutine.Config(), elevatorSimulation);
        ElevatorRequests firstMotorRequests = new ElevatorRequests(
                Phoenix6RequestBuilder.build(new PositionVoltage(0)),
                Phoenix6RequestBuilder.build(new VoltageOut(0))
        );
        ElevatorSignals firstMotorSignals = new ElevatorSignals(
                Phoenix6SignalBuilder.generatePhoenix6Signal(firstMotor.getDevice().getPosition(), DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS),
                Phoenix6SignalBuilder.generatePhoenix6Signal(firstMotor.getDevice().getMotorVoltage(), DEFAULT_SIGNALS_FREQUENCY_HERTZ)
        );
        ElevatorMotorStuff firstMotorStuff = new ElevatorMotorStuff(
                firstMotor,
                firstMotorRequests,
                firstMotorSignals
        );

        SuppliedDigitalInput digitalInput = new SuppliedDigitalInput(() -> false, new Debouncer(LIMIT_SWITCH_DEBOUNCE_TIME));

        return new Elevator(
                logPath,
                logPath + "LimitSwitch/",
                firstMotorStuff,
                firstMotorStuff,
                digitalInput
        );
    }

}