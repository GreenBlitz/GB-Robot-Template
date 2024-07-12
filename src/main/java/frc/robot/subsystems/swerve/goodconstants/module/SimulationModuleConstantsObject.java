package frc.robot.subsystems.swerve.goodconstants.module;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModuleConfigObject;
import frc.utils.Conversions;

public class SimulationModuleConstantsObject {

    private final double wheelDiameter;
    private final Rotation2d maxVelocityPerSecond;

    private final boolean enableFocSteer;
    private final boolean enableFocDrive;

    private final SimulationModuleConfigObject moduleConfigObject;

    public SimulationModuleConstantsObject(
            double wheelDiameter,
            double maxVelocityMetersPerSecond,
            DCMotorSim steerMotor,
            DCMotorSim driveMotor,
            boolean enableFocSteer,
            boolean enableFocDrive,
            PIDConstants steerMotorPIDConstants
    ){
        this.wheelDiameter = wheelDiameter;
        this.maxVelocityPerSecond = Conversions.distanceToAngle(maxVelocityMetersPerSecond, wheelDiameter);

        this.enableFocSteer = enableFocSteer;
        this.enableFocDrive = enableFocDrive;

        TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
        steerMotorConfig.Slot0.kP = steerMotorPIDConstants.kP;
        steerMotorConfig.Slot0.kI = steerMotorPIDConstants.kI;
        steerMotorConfig.Slot0.kD = steerMotorPIDConstants.kD;
        steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        this.moduleConfigObject = new SimulationModuleConfigObject(
                new SimpleMotorSimulation(steerMotor),
                new SimpleMotorSimulation(driveMotor),
                steerMotorConfig
        );
    }

    public boolean enableFocSteer(){
        return enableFocSteer;
    }

    public boolean enableFocDrive(){
        return enableFocDrive;
    }

    public double wheelDiameter() {
        return wheelDiameter;
    }

    public Rotation2d maxVelocityPerSecond() {
        return maxVelocityPerSecond;
    }

    public SimulationModuleConfigObject moduleConfigObject() {
        return moduleConfigObject;
    }

}
