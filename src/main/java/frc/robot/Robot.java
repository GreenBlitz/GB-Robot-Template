// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);
    public final Arm arm;
	public Robot() {
		BatteryUtil.scheduleLimiter();
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.SensorToMechanismRatio = 450.0 / 7.0;
        feedbackConfigs.RotorToSensorRatio = 1;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 70;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kS = 0;
        slot0Configs.kG = 0;
        ArmSimulationConstants simulationConstants = new ArmSimulationConstants(Rotation2d.fromDegrees(17),Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(17),0.001,0.3);
        this.arm = TalonFXArmBuilder.buildArm(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX+"/arm",new Phoenix6DeviceID(1),false,new TalonFXFollowerConfig(),new SysIdRoutine.Config(),feedbackConfigs,slot0Configs,slot0Configs,40,50,0,simulationConstants);
	}

	public void periodic() {
		BusChain.refreshAll();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

    public Arm getArm(){
        return arm;
    }

}
