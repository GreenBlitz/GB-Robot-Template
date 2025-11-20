// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.constants.fourBar.FourBarConstants;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.KrakenX60FlyWheelBuilder;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);;
	private final FlyWheel flyWheel;
	private final Arm fourBar;

	public Robot() {
		BatteryUtil.scheduleLimiter();
		this.flyWheel = KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);
		this.fourBar = createFourBar();
	}

	public void periodic() {
		BusChain.refreshAll();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public FlyWheel getFlyWheel() {
		return flyWheel;
	}

	public Arm createFourBar(){
		return TalonFXArmBuilder.buildDynamicMotionMagicArm(FourBarConstants.LOG_PATH,IDs.TalonFXIDs.FOUR_BAR,false,new TalonFXFollowerConfig(),new SysIdRoutine.Config(),FourBarConstants.FEEDBACK_CONFIGS,FourBarConstants.REAL_SLOTS_CONFIGS,FourBarConstants.SIMULATION_SLOTS_CONFIGS,FourBarConstants.CURRENT_LIMIT,RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,FourBarConstants.MOMENT_OF_INERTIA,FourBarConstants.ARM_LENGTH_METERS,0,FourBarConstants.FORWARD_SOFTWARE_LIMITS,FourBarConstants.BACKWARD_SOFTWARE_LIMITS,FourBarConstants.MAX_ACCELERATION_ROTATION2D_METERS_PER_SECONDS_SQUARE,FourBarConstants.MAX_VELOCITY_ROTATION2D_METERS_PER_SECONDS);
	}
	public Arm getFourBar(){
		return fourBar;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}
