package frc.robot.subsystems.funnel;

public enum FunnelState {

    PRE_SPEAKER(0.5),
    SPEAKER(0.6),
    PRE_AMP(-0.5),
    AMP(-0.6),
    ELEVATOR_TO_SHOOTER(0.5);

    private final double power;

    FunnelState(double power){
        this.power = power;
    }

    public double getPower(){
        return power;
    }

}
