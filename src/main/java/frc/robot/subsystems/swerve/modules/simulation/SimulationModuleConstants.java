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

    private final boolean enableFocSteer;
    private final boolean enableFocDrive;

    private final SimpleMotorSimulation steerMotor;
    private final SimpleMotorSimulation driveMotor;

    public SimulationModuleConstants(
            double wheelDiameterMeters,
            double maxVelocityMetersPerSecond,
            DCMotorSim steerMotor,
            DCMotorSim driveMotor,
            boolean enableFocSteer,
            boolean enableFocDrive,
            PIDConstants steerMotorPIDConstants
    ){
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.maxVelocityPerSecond = Conversions.distanceToAngle(maxVelocityMetersPerSecond, wheelDiameterMeters);

        this.enableFocSteer = enableFocSteer;
        this.enableFocDrive = enableFocDrive;

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
        this.steerMotor = moduleConfigObject.steerMotor();
        this.driveMotor = moduleConfigObject.driveMotor();
    }

    public boolean enableFocSteer(){
        return enableFocSteer;
    }

    public boolean enableFocDrive(){
        return enableFocDrive;
    }

    public double wheelDiameterMeters() {
        return wheelDiameterMeters;
    }

    public Rotation2d maxVelocityPerSecond() {
        return maxVelocityPerSecond;
    }

    public SimpleMotorSimulation steerMotor() {
        return steerMotor;
    }

    public SimpleMotorSimulation driveMotor() {
        return driveMotor;
    }

}
