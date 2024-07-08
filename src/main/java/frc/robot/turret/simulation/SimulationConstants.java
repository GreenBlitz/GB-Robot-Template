package frc.robot.turret.simulation;

import edu.wpi.first.math.controller.PIDController;

public class SimulationConstants {
    public static double GEARING = 1/100.0;
    public static int AMOUNT_OF_MOTORS = 1;
    public static double JKG_METER_SQ = 0.003;
    public static PIDController VELOCITY_PID_CONTROLLER = new PIDController(1,0,0);
    public static PIDController POSITION_PID_CONTROLLER = new PIDController(1,0,0);
}
