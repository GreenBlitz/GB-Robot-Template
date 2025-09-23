// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
    public static final boolean isMecanum = true;

    private final SparkMaxWrapper intake;
    private MecanumDrive mecanumDrive;
    private DifferentialDrive tankDrive;

	public Robot() {
		BatteryUtil.scheduleLimiter();
        intake = new SparkMaxWrapper(new SparkMaxDeviceID(0));
        if (isMecanum) {
            SparkMaxWrapper frontLeft = new SparkMaxWrapper(new SparkMaxDeviceID(0));
            SparkMaxWrapper rearLeft = new SparkMaxWrapper(new SparkMaxDeviceID(0));
            SparkMaxWrapper frontRight = new SparkMaxWrapper(new SparkMaxDeviceID(0));
            SparkMaxWrapper rearRight = new SparkMaxWrapper(new SparkMaxDeviceID(0));
            mecanumDrive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);
        } else {
        SparkMaxWrapper leftTank = new SparkMaxWrapper(new SparkMaxDeviceID(0));
        SparkMaxWrapper rightTank = new SparkMaxWrapper(new SparkMaxDeviceID(0));
        tankDrive = new DifferentialDrive(leftTank::set, rightTank::set);
        }
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

    public SparkMaxWrapper getIntake() {
        return intake;
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public DifferentialDrive getTankDrive() {
        return tankDrive;
    }

}
