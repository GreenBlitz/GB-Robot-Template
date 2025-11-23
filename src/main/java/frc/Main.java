// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what you are doing, do not modify this file
 * except to change the parameter class to the startRobot call.
 */
public final class Main {

	/**
	 * Main initialization function. Do not perform any initialization here.
	 *
	 * <p>If you change your main robot class, change the parameter type.
	 */
	public static void main(String... args) {
		RobotBase.startRobot(RobotManager::new);
//		Pose3d pose1 = new Pose3d(
//			0,
//			0,
//			0,
//			new Rotation3d(
//				Rotation2d.fromDegrees(45).getRadians(),
//				Rotation2d.fromDegrees(45).getRadians(),
//				Rotation2d.fromDegrees(45).getRadians()
//			)
//		);
//		Pose3d pose2 = new Pose3d(
//			0,
//			0,
//			0,
//			new Rotation3d(
//				Rotation2d.fromDegrees(90).getRadians(),
//				Rotation2d.fromDegrees(90).getRadians(),
//				Rotation2d.fromDegrees(90).getRadians()
//			)
//		);
//		double sinX = Math.sin(pose1.getRotation().getX()) + Math.sin(pose2.getRotation().getX());
//		double cosX = Math.cos(pose1.getRotation().getX()) + Math.cos(pose2.getRotation().getX());
//		double sinY = Math.sin(pose1.getRotation().getY()) + Math.sin(pose2.getRotation().getY());
//		double cosY = Math.cos(pose1.getRotation().getY()) + Math.cos(pose2.getRotation().getY());
//		double sinZ = Math.sin(pose1.getRotation().getZ()) + Math.sin(pose2.getRotation().getZ());
//		double cosZ = Math.cos(pose1.getRotation().getZ()) + Math.cos(pose2.getRotation().getZ());
//		Rotation3d finalPose = AngleMath.getAngleAverageWrapped(sinX, cosX, sinY, cosY, sinZ, cosZ, 2);
//		System.out.println(
//			Rotation2d.fromRadians(finalPose.getX()).getDegrees()
//				+ " "
//				+ Rotation2d.fromRadians(finalPose.getY()).getDegrees()
//				+ " "
//				+ Rotation2d.fromRadians(finalPose.getZ()).getDegrees()
//		);
	}

}
