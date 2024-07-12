package frc.robot.subsystems.swerve.modules.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.utils.Conversions;

public class SimulationModuleConstants {

    private final double wheelDiameterMeters;
    private final Rotation2d maxVelocityPerSecond;

    private final boolean enableFOCSteer;
    private final boolean enableFOCDrive;

    private final SimpleMotorSimulation steerMotor;
    private final SimpleMotorSimulation driveMotor;

    public SimulationModuleConstants(
            double wheelDiameterMeters,
            double maxVelocityMetersPerSecond,
            DCMotorSim steerMotor,
            DCMotorSim driveMotor,
            boolean enableFOCSteer,
            boolean enableFOCDrive,
            PIDConstants steerMotorPIDConstants
    ){
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.maxVelocityPerSecond = Conversions.distanceToAngle(maxVelocityMetersPerSecond, wheelDiameterMeters);

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

    protected boolean getEnableFocSteer(){
        return enableFOCSteer;
    }

    protected boolean getEnableFocDrive(){
        return enableFOCDrive;
    }

    protected double getWheelDiameterMeters() {
        return wheelDiameterMeters;
    }

    protected Rotation2d getMaxVelocityPerSecond() {
        return maxVelocityPerSecond;
    }

    protected SimpleMotorSimulation getSteerMotor() {
        return steerMotor;
    }

    protected SimpleMotorSimulation getDriveMotor() {
        return driveMotor;
    }

}
