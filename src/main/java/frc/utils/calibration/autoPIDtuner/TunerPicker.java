package frc.utils.calibration.autoPIDtuner;

import frc.utils.calibration.autoPIDtuner.PIDObject;
public class TunerPicker {

    private ControllerType controllerType;
    AutoPIDTuner autoPIDTuner;

    public TunerPicker(ControllerType controllerType) {
        this.controllerType = controllerType;
        this.autoPIDTuner = new AutoPIDTuner();
    }

    public PIDObject tune(double ku) {
        return autoPIDTuner.tunePController(ku);
    }

    public PIDObject tune(double ku, double pu) {
        return controllerType==ControllerType.PI_CONTROLLER ?
                autoPIDTuner.tunePIController(ku, pu)
                : autoPIDTuner.tunePIDController(ku, pu);
    }

}
