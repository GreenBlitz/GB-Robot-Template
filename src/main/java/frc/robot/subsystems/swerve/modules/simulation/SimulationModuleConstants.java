package frc.robot.subsystems.swerve.modules.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.utils.Conversions;

public class SimulationModuleConstants {

    private final double wheelDiameterMeters;
    private final Rotation2d velocityAt12VoltsPerSecond;

    private final boolean enableFOCSteer;
    private final boolean enableFOCDrive;

    private final SimpleMotorSimulation steerMotor;
    private final SimpleMotorSimulation driveMotor;

    public SimulationModuleConstants(
            double wheelDiameterMeters,
            double velocityAt12VoltsMetersPerSecond,
            DCMotorSim steerMotor,
            DCMotorSim driveMotor,
            boolean enableFOCSteer,
            boolean enableFOCDrive,
            PIDConstants steerMotorPIDConstants
    ){
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.velocityAt12VoltsPerSecond = Conversions.distanceToAngle(velocityAt12VoltsMetersPerSecond, wheelDiameterMeters);

        this.enableFOCSteer = enableFOCSteer;
        this.enableFOCDrive = enableFOCDrive;

        TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
        steerMotorConfig.Slot0.kP = steerMotorPIDConstants.kP;
        steerMotorConfig.Slot0.kI = steerMotorPIDConstants.kI;
        steerMotorConfig.Slot0.kD = steerMotorPIDConstants.kD;
        steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        SimulationModuleConfigObject moduleConfigObject = new SimulationModuleConfigObject(
                new SimpleMotorSimulation(steerMotor),
                new SimpleMotorSimulation(driveMotor),
                steerMotorConfig
        );
        this.steerMotor = moduleConfigObject.getSteerMotor();
        this.driveMotor = moduleConfigObject.getDriveMotor();
    }

    protected double getWheelDiameterMeters() {
        return wheelDiameterMeters;
    }

    protected Rotation2d getVelocityAt12VoltsPerSecond() {
        return velocityAt12VoltsPerSecond;
    }

    protected boolean getEnableFOCSteer(){
        return enableFOCSteer;
    }

    protected boolean getEnableFOCDrive(){
        return enableFOCDrive;
    }

    protected SimpleMotorSimulation getSteerMotor() {
        return steerMotor;
    }

    protected SimpleMotorSimulation getDriveMotor() {
        return driveMotor;
    }

}
