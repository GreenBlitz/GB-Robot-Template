package frc.utils.joysticks;

enum ButtonID {

    A(1),
    B(2),
    X(3),
    Y(4),

    L1(5),
    R1(6),

    BACK(7),
    START(8),

    L3(9),
    R3(10),

    POV_UP(0),
    POV_RIGHT(90),
    POV_DOWN(180),
    POV_LEFT(270);

    private final int id;

    ButtonID(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }

}
