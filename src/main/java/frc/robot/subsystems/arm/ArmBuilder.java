package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public class ArmBuilder {
    public Arm create(String logPath, int armId, int followerId, double JKgMetersSquared, double gearing, boolean opposeMain, Rotation2d maxAcceleration, Rotation2d maxVelocity,double feedForward){
        TalonFXFollowerConfig followerConfig = new TalonFXFollowerConfig();
        followerConfig.followerIDs = new TalonFXFollowerConfig.TalonFXFollowerID[]{new TalonFXFollowerConfig.TalonFXFollowerID("ArmFollower",new Phoenix6DeviceID(followerId),opposeMain)};
        TalonFXMotor arm = new TalonFXMotor(
                logPath + "/Arm",
                new Phoenix6DeviceID(armId),
                new SysIdRoutine.Config(),
                new SimpleMotorSimulation(
                        new DCMotorSim(
                                LinearSystemId.createDCMotorSystem(
                                        DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length+1),
                                        JKgMetersSquared,
                                        gearing
                                ),
                                DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length+1)
                        )
                )
        );


        Phoenix6AngleSignal velocity = Phoenix6SignalBuilder
                .build(arm.getDevice().getVelocity(), ArmConstants.defaultFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);

        Phoenix6LatencySignal position = Phoenix6SignalBuilder
                .build(arm.getDevice().getPosition(), velocity,ArmConstants.defaultFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);

        Phoenix6DoubleSignal voltage = Phoenix6SignalBuilder.build(arm.getDevice().getMotorVoltage(), ArmConstants.defaultFrequency, BusChain.ROBORIO);

        Phoenix6DoubleSignal current = Phoenix6SignalBuilder.build(arm.getDevice().getStatorCurrent(), ArmConstants.defaultFrequency, BusChain.ROBORIO);

        Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0),true);

        IDynamicMotionMagicRequest positionRequest = Phoenix6RequestBuilder.build(new DynamicMotionMagicVoltage(position.getLatestValue().getRotations(),0,0,0),0,true);
        positionRequest.withMaxAccelerationRotation2dPerSecondSquared(maxAcceleration);
        positionRequest.withMaxVelocityRotation2dPerSecond(maxVelocity);
        positionRequest.withArbitraryFeedForward(feedForward);
        return new Arm(logPath,arm,velocity,position,voltage,current,voltageRequest,positionRequest);
    }


    public Arm create(String logPath,int armId,int followerId,double gearing,double JKgMetersSquared,boolean opposeMain,double feedforward){
        TalonFXFollowerConfig followerConfig = new TalonFXFollowerConfig();
        followerConfig.followerIDs = new TalonFXFollowerConfig.TalonFXFollowerID[]{new TalonFXFollowerConfig.TalonFXFollowerID("ArmFollower",new Phoenix6DeviceID(followerId),opposeMain)};
        TalonFXMotor arm = new TalonFXMotor(
                logPath + "/Arm",
                new Phoenix6DeviceID(armId),
                new SysIdRoutine.Config(),
                new SimpleMotorSimulation(
                        new DCMotorSim(
                                LinearSystemId.createDCMotorSystem(
                                        DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length+1),
                                        JKgMetersSquared,
                                        gearing
                                ),
                                DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length+1)
                        )
                )
        );

        Phoenix6AngleSignal velocity = Phoenix6SignalBuilder
                .build(arm.getDevice().getVelocity(), 50, AngleUnit.ROTATIONS, BusChain.ROBORIO);

        Phoenix6LatencySignal position = Phoenix6SignalBuilder
                .build(arm.getDevice().getPosition(), velocity,50, AngleUnit.ROTATIONS, BusChain.ROBORIO);

        Phoenix6DoubleSignal voltage = Phoenix6SignalBuilder.build(arm.getDevice().getMotorVoltage(), 50, BusChain.ROBORIO);

        Phoenix6DoubleSignal current = Phoenix6SignalBuilder.build(arm.getDevice().getStatorCurrent(), 50, BusChain.ROBORIO);

        Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0),true);

        Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder.build(new MotionMagicVoltage(position.getLatestValue().getRotations()),feedforward,true);
        return new Arm(logPath,arm,velocity,position,voltage,current,voltageRequest,positionRequest);
    }

}
