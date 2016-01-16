package phys2d.collisionLogic.collisionCheckers;

import phys2d.entities.shapes.Shape;

/**
 * This makes sure that the implementing class supports swept collision
 * detection. By returning the collision time, the collision manager is able to
 * take the appropriate action to deal with collision in a continuous manner.
 * 
 * @author afsheen
 *
 */
public interface SweptCapable {

    /**
     * Checks to see if there will be a collision between these two shapes in
     * the next frame. If there is, return the exact time during the next frame
     * that the collision should happen.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param collInfo the result of the collision detection algorithm.
     * @param dt the physics delta time of the simulation.
     * 
     * @return a double between 0.0-1.0 representing when a collision will take
     *         place next frame. If there is no collision next frame, return -1.
     */
    public double getImpendingCollisionTime(Shape s1, Shape s2, CollisionInfo collInfo, double dt);

}
