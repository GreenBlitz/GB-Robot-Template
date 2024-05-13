package frc.utils.mirrorutils;

import frc.utils.DriverStationUtils;

/**
 * A class that allows for objects to be mirrored across the center of the field when the robot is on the red alliance.
 * This is useful for placing field elements and other objects that are mirrored across the field, or for mirroring the target
 * heading to face a field element.
 *
 * @param <T> the type of object to mirror
 */
public abstract class Mirrorable<T> {

    protected final T nonMirroredObject, mirroredObject;

    protected final boolean mirrorWhenRedAlliance;

    /**
     * Creates a new mirrorable object.
     *
     * @param nonMirroredObject the object when the robot is on the blue alliance, or the non-mirrored object
     * @param mirrorWhenRedAlliance whether to mirror the object when the robot is on the red alliance
     */
    protected Mirrorable(T nonMirroredObject, boolean mirrorWhenRedAlliance) {
        this.nonMirroredObject = nonMirroredObject;
        this.mirroredObject = mirror(nonMirroredObject);
        this.mirrorWhenRedAlliance = mirrorWhenRedAlliance;
    }

    /**
     * @return the current object.
     * If the robot is on the red alliance and the object should be mirrored, the mirrored object is returned.
     * Otherwise, the non-mirrored object is returned.
     */
    public T get() {
        return DriverStationUtils.isRedAlliance() && mirrorWhenRedAlliance ? mirroredObject : nonMirroredObject;
    }

    /**
     * Mirrors the object across the center of the field.
     *
     * @param object the object to mirror
     * @return the mirrored object
     */
    protected abstract T mirror(T object);

}
