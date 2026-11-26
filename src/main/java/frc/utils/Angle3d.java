package frc.utils;

import frc.utils.angle2d.Angle2d;

public class Angle3d {

	private Angle2d roll;
	private Angle2d pitch;
	private Angle2d yow;

	public Angle3d(Angle2d roll, Angle2d pitch, Angle2d yow) {
		this.roll = roll;
		this.pitch = pitch;
		this.yow = yow;
	}

	public Angle3d(Angle2d[] angle2ds) {
		this.roll = angle2ds[0];
		this.pitch = angle2ds[1];
		this.yow = angle2ds[2];
	}

	public Angle2d getRoll() {
		return roll;
	}

	public Angle2d getPitch() {
		return pitch;
	}

	public Angle2d getYow() {
		return yow;
	}

}
