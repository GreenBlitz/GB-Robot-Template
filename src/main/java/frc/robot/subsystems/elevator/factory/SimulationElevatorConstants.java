package frc.robot.subsystems.elevator.factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.mechanisms.wpilib.ElevatorSimulation;
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

    private static final double RADIUS_METERS = 0.025;
    private static final int NUMBER_OF_MOTORS = 2;
    private static final double MIN_HEIGHT_METERS = 0;
    private static final double MAX_HEIGHT_METERS = 2;
    private static final double STARTING_HEIGHT_METERS = 0;
    private static final double GEAR_RATIO = 1.0 / 5.0;
    private static final double CURRENT_LIMIT = 40;
    private static final boolean CURRENT_LIMIT_ENABLE = true;
    private static final boolean SOFT_LIMIT_ENABLE = true;
    private static final SysIdRoutine.Config MOTOR_CONFIG = new SysIdRoutine.Config();

    private static final double KP = 1;
    private static final double KI = 1;
    private static final double KD = 1;

    private static void configMotor(TalonFXMotor motor){
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Slot0.withKP(KP).withKI(KI).withKD(KD);
        configuration.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        configuration.CurrentLimits.StatorCurrentLimitEnable = CURRENT_LIMIT_ENABLE;
        configuration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(frc.robot.subsystems.elevator.factory.ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS);
        configuration.SoftwareLimitSwitch.withReverseSoftLimitEnable(SOFT_LIMIT_ENABLE);
        motor.applyConfiguration(configuration);
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
        TalonFXMotor firstMotor = new TalonFXMotor(logPath + "FirstMotor/", IDs.Phoenix6IDs.ELEVATOR_FIRST_MOTOR_ID, MOTOR_CONFIG, elevatorSimulation);
        ElevatorRequests firstMotorRequests = createRequests();
        ElevatorSignals firstMotorSignals = createSignals(firstMotor);
        configMotor(firstMotor);
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