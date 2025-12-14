// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Rotation2d;
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
//		RobotBase.startRobot(RobotManager::new);
        Rotation2d rot1 = Rotation2d.fromDegrees(145);
        Rotation2d rot2 = Rotation2d.fromDegrees(0);
        Rotation2d rot3 = Rotation2d.fromDegrees(180);

        double sumSin = rot1.getSin() + rot2.getSin() + rot3.getSin();
//        System.out.println(rot1.getSin());
//        System.out.println(rot2.getSin());
//        System.out.println(rot3.getSin());
        double sumCos = rot1.getCos() + rot2.getCos() + rot3.getCos();
//        System.out.println(rot1.getCos());
//        System.out.println(rot2.getCos());
//        System.out.println(rot3.getCos());

        Rotation2d avg = new Rotation2d(sumSin / 3.0, sumCos / 3.0);
        System.out.println(avg);

    }

}
