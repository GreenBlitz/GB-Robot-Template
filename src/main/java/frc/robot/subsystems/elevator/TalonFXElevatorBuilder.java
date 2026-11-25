package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.interfaces.VelocityPositionRequest;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.hardware.simulations.MechanismSimulation;
import frc.robot.hardware.simulations.wpilib.ElevatorSimulation;
import frc.robot.hardware.simulations.wpilib.FlywheelSimulation;
import frc.utils.AngleUnit;

public class TalonFXElevatorBuilder {
    public static Elevator buildElevator(
            String logPath,
            Phoenix6DeviceID deviceID,
            ElevatorPhysicalProperties physicalProperties,
            Distance startingHeight,
            Slot0Configs realVelocityControlConfig,
            Slot0Configs simulationVelocityControlConfig,
            TalonFXFollowerConfig talonFXFollowerConfig,
            SysIdRoutine.Config sysIdRoutineConfig,
            int currentLimit,
            FeedbackConfigs feedbackConfigs,
            double massKg,
            double radiusMeters,
            boolean isInverted,
            boolean isFlywheel
    ){
        MechanismSimulation motorSimulation=new ElevatorSimulation(
                new ElevatorSim(
                        LinearSystemId.createElevatorSystem(
                                DCMotor.getKrakenX60(talonFXFollowerConfig.followerIDs.length+1),massKg,radiusMeters,
                                feedbackConfigs.RotorToSensorRatio* feedbackConfigs.SensorToMechanismRatio
                        ),DCMotor.getKrakenX60(talonFXFollowerConfig.followerIDs.length+1),physicalProperties.elevatorMinHeight().in(Units.Meter),physicalProperties.elevatorMinHeight().plus(physicalProperties.elevatorHeight()).in(Units.Meter),true,startingHeight.in(Units.Meter)

                ),radiusMeters*2,feedbackConfigs.RotorToSensorRatio* feedbackConfigs.SensorToMechanismRatio
        );

        TalonFXMotor motor = new TalonFXMotor(logPath,deviceID,talonFXFollowerConfig,sysIdRoutineConfig);
        double maxPosition = Math.max(physicalProperties.motorMinPosition().getRotations(),physicalProperties.motorMaxPosition().getRotations());
        double minPosition = Math.min(physicalProperties.motorMinPosition().getRotations(),physicalProperties.motorMaxPosition().getRotations());
        motor.applyConfiguration(
                buildConfiguration(isInverted,feedbackConfigs,currentLimit,
                        RobotManager.isReal()?realVelocityControlConfig:simulationVelocityControlConfig,
                        maxPosition,minPosition
                )
        );

        InputSignal<Double> voltageSignal = Phoenix6SignalBuilder
                .buildDoubleSignal(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, deviceID.busChain());
        InputSignal<Double> currentSignal = Phoenix6SignalBuilder
                .buildDoubleSignal(motor.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, deviceID.busChain());
        InputSignal<Rotation2d> positionSignal = Phoenix6SignalBuilder.buildLatencySignal(
                motor.getDevice().getPosition(),
                motor.getDevice().getVelocity(),
                RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                AngleUnit.ROTATIONS,
                deviceID.busChain()
        );
        InputSignal<Rotation2d> velocitySignal = Phoenix6SignalBuilder.buildAngleSignal(
                motor.getDevice().getVelocity(),
                RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                AngleUnit.ROTATIONS,
                deviceID.busChain()
        );
        VelocityPositionRequest velocityPositionRequest = Phoenix6RequestBuilder.buildVelocityPositionRequest(new PositionVoltage(0),1,true);
        IRequest<Double> voltageRequest = Phoenix6RequestBuilder.buildVoltageOutRequest(new VoltageOut(0),true);
        return new Elevator(logPath,motor,voltageSignal,currentSignal,positionSignal,velocitySignal,velocityPositionRequest,voltageRequest,physicalProperties);

    }

    public static TalonFXConfiguration buildConfiguration(
            boolean inverted,
            FeedbackConfigs feedbackConfigs,
            int currentLimit,
            Slot0Configs ConfigurationSlots,
            double maxPositionRots,
            double minPositionRots
    ) {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0 = ConfigurationSlots;
        configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.Feedback = feedbackConfigs;
        configs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold=maxPositionRots;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable=true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable=true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold=minPositionRots;
        return (configs);
    }
}
