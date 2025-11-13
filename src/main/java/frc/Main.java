// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.*;
import frc.utils.math.AngleMath;

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
		Pose3d cam = new Pose3d(-2, 0, 0, new Rotation3d(0, 0, 0));
		Pose3d tag = new Pose3d(-1, 1, 0, new Rotation3d(0, 0, Math.PI));
		Pose3d bot = new Pose3d();

		Pose3d camToTagRelToZeroDeg = new Pose3d(-1, -1, 0, new Rotation3d(0, 0, Math.PI));
		Pose3d camToTagRelToTagDeg = new Pose3d(1, 1, 0, new Rotation3d(0, 0, Math.PI));

//        Transform3d bad = new Transform3d(new Pose3d(), camToTagRelToTagDeg);
//        Transform3d good = new Transform3d(tag, cam);

		Pose3d bad = AngleMath.transformBy(tag, camToTagRelToZeroDeg);
		Pose3d good = tag.transformBy(new Transform3d(tag, cam));

		System.out.println(bad.getX());
		System.out.println(bad.getY());
		System.out.println(bad.getZ());
		System.out.println(Rotation2d.fromRadians(bad.getRotation().getX()).getDegrees());
		System.out.println(Rotation2d.fromRadians(bad.getRotation().getY()).getDegrees());
		System.out.println(Rotation2d.fromRadians(bad.getRotation().getZ()).getDegrees());

		System.out.println("");

		System.out.println(good.getX());
		System.out.println(good.getY());
		System.out.println(good.getZ());
		System.out.println(Rotation2d.fromRadians(good.getRotation().getX()).getDegrees());
		System.out.println(Rotation2d.fromRadians(good.getRotation().getY()).getDegrees());
		System.out.println(Rotation2d.fromRadians(good.getRotation().getZ()).getDegrees());
	}

}
