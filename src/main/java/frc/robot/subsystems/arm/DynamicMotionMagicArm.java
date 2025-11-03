package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.*;

import java.lang.module.Configuration;

public class DynamicMotionMagicArm extends Arm{

    private final IDynamicMotionMagicRequest motionMagicRequest;
    private final Rotation2d defaultDynamicMotionAcceleration;
    private final Rotation2d defaultDynamicMotionVelocity;


    public DynamicMotionMagicArm(String logPath, ControllableMotor arm, InputSignal<Rotation2d> velocitySignal, InputSignal<Rotation2d> positionSignal, InputSignal<Double> voltageSignal, InputSignal<Double> currentSignal, IRequest<Double> armVoltageRequest, IDynamicMotionMagicRequest motionMagicRequest, Rotation2d defaultMotionMagicAcceleration, Rotation2d defaultMotionMagicVelocity) {
        super(logPath, arm, velocitySignal, positionSignal, voltageSignal, currentSignal, armVoltageRequest, motionMagicRequest);
        this.motionMagicRequest = motionMagicRequest;
        this.defaultDynamicMotionAcceleration = defaultMotionMagicAcceleration;
        this.defaultDynamicMotionVelocity = defaultMotionMagicVelocity;
    }

    public void withPosition(Rotation2d target,Rotation2d acceleration,Rotation2d velocity,double arbitraryFeedForward){
        motionMagicRequest.withSetPoint(target);
        motionMagicRequest.withMaxAccelerationRotation2dPerSecondSquared(acceleration);
        motionMagicRequest.withMaxVelocityRotation2dPerSecond(velocity);
        motionMagicRequest.withArbitraryFeedForward(arbitraryFeedForward);
    }

    public void withPosition(Rotation2d target){
        motionMagicRequest.withSetPoint(target);
        motionMagicRequest.withMaxAccelerationRotation2dPerSecondSquared(defaultDynamicMotionAcceleration);
        motionMagicRequest.withMaxVelocityRotation2dPerSecond(defaultDynamicMotionVelocity);
    }




}
