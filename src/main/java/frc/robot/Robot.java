// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmBuilder;
import frc.robot.subsystems.arm.DynamicMotionMagicArm;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
    private static final TalonFXConfiguration config = new TalonFXConfiguration();



    private final DynamicMotionMagicArm arm;
//    private final Arm pivot;

	public Robot() {
		BatteryUtil.scheduleLimiter();
        config.Slot0.kP = 70;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kG = 0;

        config.Slot1.kP = 70;
        config.Slot1.kI = 0;
        config.Slot1.kD = 0;
        config.Slot1.kS = 0;
        config.Slot1.kG = 0;

        config.MotionMagic.withMotionMagicCruiseVelocity(3);
        config.MotionMagic.withMotionMagicAcceleration(3);
        arm = ArmBuilder.createDynamicMotionMagic("Arm/",new TalonFXFollowerConfig() ,20,Volts.of(7),null,Volts.of(1).per(Second),0.001, SingleJointedArmSim.estimateMOI(0.3, 5),Rotation2d.fromRotations(3),Rotation2d.fromRotations(3),0,450.0 / 7.0,1,config,config,0.37,0.2);
//        pivot = ArmBuilder.create("Pivot/",new SysIdRoutine.Config(),)
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
    public DynamicMotionMagicArm getArm(){
        return arm;
    }

}
