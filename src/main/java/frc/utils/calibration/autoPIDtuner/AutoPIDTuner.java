package frc.utils.calibration.autoPIDtuner;

import frc.utils.calibration.autoPIDtuner.PIDObject;
public class AutoPIDTuner {

    public PIDObject tunePController(double ku) {
        return new PIDObject(ku/AutoPIDTunerConstants.P_CONTROLLER.DIVIDE_KU_TO_GET_KP);
    }

    public PIDObject tunePIController(double ku, double pu) {
        return new PIDObject(ku / AutoPIDTunerConstants.PI_CONTROLLER.DIVIDE_KU_TO_GET_KP,
                pu / AutoPIDTunerConstants.PI_CONTROLLER.DIVIDE_PU_TO_GET_KI);
    }

    public PIDObject tunePIDController(double ku, double pu) {
        return new PIDObject(ku / AutoPIDTunerConstants.PID_CONTROLLER.DIVIDE_KU_TO_GET_KP,
                pu / AutoPIDTunerConstants.PID_CONTROLLER.DIVIDE_PU_TO_GET_KI,
                pu / AutoPIDTunerConstants.PID_CONTROLLER.DIVIDE_PU_TO_GET_KD);
    }

}