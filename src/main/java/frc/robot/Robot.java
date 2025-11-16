// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.constants.hood.Constants;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);
    private final Arm hood;

	public Robot() {
		BatteryUtil.scheduleLimiter();
        FeedbackConfigs hoodFeedbackConfig = new FeedbackConfigs();

        hoodFeedbackConfig.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
        hoodFeedbackConfig.RotorToSensorRatio = 1;

        hood = TalonFXArmBuilder.buildMotionMagicArm(
                RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Hood",
                new Phoenix6DeviceID(IDs.TalonFXIDs.hoodId,BusChain.ROBORIO),
                HoodConstants.isInverted,
                new TalonFXFollowerConfig(),
                new SysIdRoutine.Config(), //q
                hoodFeedbackConfig,


                )
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

    public Arm getHood() {
        return hood;
    }
}
