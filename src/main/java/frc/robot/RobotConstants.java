package frc.robot;

import frc.utils.math.StandardDeviations2D;

public class RobotConstants {

	public static final double DEFAULT_SIGNALS_FREQUENCY_HERTZ = 60;

	public static final String SUBSYSTEM_LOGPATH_PREFIX = "Subsystems";

    public static final double ROBOT_MASS_KG = 45;
    public static final double ROBOT_MOI = 4.5;
    public static final double WHEEL_COF = 1;

    public static final StandardDeviations2D DEFAULT_TAG_DISTANCE_FACTORS = new StandardDeviations2D(0.5);
    public static final StandardDeviations2D DEFAULT_STD_DEV_FACTORS = new StandardDeviations2D(0.15);
    public static final StandardDeviations2D DEFAULT_VISIBLE_TAGS_EXPONENTS = new StandardDeviations2D(0.4);
    public static final StandardDeviations2D DEFAULT_STD_DEV_ADDITIONS = new StandardDeviations2D(0.011);

    // data fetched from:
	// https://www.mt.com/mt_ext_files/Editorial/Generic/7/NewtonFactor_Editorial-Generic_1149155213028_files/Newton%20Factor%20and%20Values.pdf
	public static final double G = 9.7933;

}
