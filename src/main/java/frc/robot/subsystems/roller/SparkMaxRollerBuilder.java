package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxConfiguration;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.rev.request.SparkMaxRequestBuilder;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;

import java.io.ObjectInputFilter;

public class SparkMaxRollerBuilder {

    public static Roller createSparkMaxMotorRoller (String logPath, SparkMaxDeviceID rotorID,double gearRatio,int currentLimiting){
        SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(rotorID);

        SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getNEO(1),
                                0.001,
                                gearRatio
                        ),
                        DCMotor.getNEO(1)
                )
        );

        BrushlessSparkMAXMotor roller = new BrushlessSparkMAXMotor(
                logPath,
                sparkMaxWrapper,
                rollerSimulation,
                new SysIdRoutine.Config()

        );
        SuppliedDoubleSignal voltageSignal= new SuppliedDoubleSignal("voltage",sparkMaxWrapper::getVoltage);
        SuppliedDoubleSignal currentSignal= new SuppliedDoubleSignal("voltage",sparkMaxWrapper::getOutputCurrent);
        roller.applyConfiguration(configRoller(gearRatio,currentLimiting));
        return new Roller(logPath,roller,voltageSignal,currentSignal);
    }
    public static SparkMaxConfiguration configRoller(double gearRatio, int currentLimiting){
        SparkMaxConfiguration configs= new SparkMaxConfiguration();
        configs.getSparkMaxConfig().smartCurrentLimit(currentLimiting);
        configs.getSparkMaxConfig().encoder.positionConversionFactor(gearRatio);
        configs.getSparkMaxConfig().encoder.velocityConversionFactor(gearRatio);
        return configs;
    }
}
