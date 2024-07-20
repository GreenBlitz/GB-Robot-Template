package frc.utils.calibration.autoPIDtuner;

import edu.greenblitz.robotName.utils.PIDObject;


public class AutoPIDTuner {

    public PIDObject tunePController(double ku) {
        return new PIDObject(ku / 2);
    }

    public PIDObject tunePIController(double ku, double pu) {
        return new PIDObject(ku / 2.2, pu / 1.2, 0, 0, 0);
    }

    public PIDObject tunePIDController(double ku, double pu) {
        return new PIDObject(ku / 1.7, pu / 2, pu / 8, 0, 0);
    }

}


