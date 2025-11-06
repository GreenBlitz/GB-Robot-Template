package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.hardware.rev.motors.SparkMaxConfiguration;

public class TalonFXRollerBuilder {

    public static Roller createTalonFXMotorRoller(String logPath, Phoenix6DeviceID rotorID,double gearRatio,int currentLimiting){
        SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX60(1),
                                0.001,
                                gearRatio
                        ),
                        DCMotor.getKrakenX60(1)
                )

        );
        TalonFXMotor roller = new TalonFXMotor(logPath,rotorID,new TalonFXFollowerConfig(),new SysIdRoutine.Config(),rollerSimulation);
        roller.applyConfiguration(configRoller(gearRatio,currentLimiting));
        Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder.build(roller.getDevice().getStatorCurrent(),50, BusChain.ROBORIO);
        Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder.build(roller.getDevice().getMotorVoltage(),50, BusChain.ROBORIO);
        return new Roller(logPath,roller,voltageSignal,currentSignal);

    }
    public static TalonFXConfiguration configRoller(double gearRatio, int currentLimiting){
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.CurrentLimits.StatorCurrentLimit = currentLimiting;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.Feedback.SensorToMechanismRatio = gearRatio;
        return (configs);

    }
}
