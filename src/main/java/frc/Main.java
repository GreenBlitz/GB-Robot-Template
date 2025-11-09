// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.utils.math.AngleMath;
import org.littletonrobotics.junction.Logger;

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
//		RobotBase.startRobot(RobotManager::new);
		Pose3d pose1 = new Pose3d();
		Pose3d pose2 = new Pose3d(100, 100, 100, new Rotation3d(Math.PI, Math.PI, Math.PI));
		Pose3d p = new Pose3d(pose1.getTranslation().plus(pose2.getTranslation()).div(2),
				AngleMath.getAngleAverageWrapped(
						Math.sin(pose1.getRotation().getX()) + Math.sin(pose2.getRotation().getX()),
						Math.cos(pose1.getRotation().getX()) + Math.cos(pose2.getRotation().getX()),
						Math.sin(pose1.getRotation().getY()) + Math.sin(pose2.getRotation().getY()),
						Math.cos(pose1.getRotation().getY()) + Math.cos(pose2.getRotation().getY()),
						Math.sin(pose1.getRotation().getZ()) + Math.sin(pose2.getRotation().getZ()),
						Math.cos(pose1.getRotation().getZ()) + Math.cos(pose2.getRotation().getZ()),
						2
				)
		);
		System.out.println(p.getTranslation().getX());
		System.out.println(p.getTranslation().getY());
		System.out.println(p.getTranslation().getZ());
		System.out.println(p.getRotation().getX());
		System.out.println(p.getRotation().getY());
		System.out.println(p.getRotation().getZ());
	}

}
