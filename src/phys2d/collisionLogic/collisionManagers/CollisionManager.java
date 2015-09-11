/**
 * 
 */
package phys2d.collisionLogic.collisionManagers;

import java.util.ArrayList;
import java.util.Arrays;

import phys2d.collisionLogic.spacePartitioning.BSPTree;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.WorldBound;

/**
 * @author afsheen
 *
 */
public abstract class CollisionManager {

    protected BSPTree collisionTree;
    protected final double dt;

    public CollisionManager(double dt) {
        this.dt = dt;
    }

    /**
     * Computes all collisions and they resolution forces.
     * 
     * @param entities
     */
    protected abstract void manageCollisions(ArrayList<Shape> entities);

    public abstract void runManager(ArrayList<Shape> entities);

    /**
     * Check if item is deeply contained within groups
     * 
     * @param groups the arraylist to check
     * @param item the item to check the list for
     * @return whether or not item is in group
     */
    protected boolean deepContains(ArrayList<Shape[]> groups, Shape[] item) {
        for (Shape[] group : groups) {
            if (Arrays.deepEquals(group, item))
                return true;
        }
        return false;
    }

    /**
     * Return the collision tree being used to partition the entities.
     * 
     * @return the space partitioning binary tree.
     */
    public BSPTree getCollisionTree() {
        return collisionTree;
    }

    /**
     * Move all the entities based on their properties and forces.
     * 
     * @param entities the entities to move.
     */
    protected void moveEntities(ArrayList<Shape> entities) {
        for (Shape entity : entities) { // move all entities
            if (!(entity instanceof WorldBound)) {
                // System.out.println("vel: " + entity.getVelocity());
                entity.move(dt);
            }
        }
    }

    protected void addForceOfGravity(Shape s) {
        addForceOfGravity(s, 1.0);
    }

    protected void addForceOfGravity(Shape s, double increment) {
        final double g = 100.0; // m/s^2
        Vec2D weightForce = new Vec2D(0, g);
        weightForce.scaleBy(s.getMass());
        weightForce.scaleBy(increment);
        weightForce.scaleBy(dt);
        s.addForce(weightForce);
    }

    /**
     * Add all the external forces acting on the entities. Currently only
     * gravity.
     * 
     * @param entities the entities inside the world.
     */
    protected void addWorldForces(ArrayList<Shape> entities) {
        for (Shape s : entities) {
            addForceOfGravity(s);
        }
    }

    /**
     * Add in a certain percent of the world forces. Used by the swept checker
     * to advance the entity by subframes.
     * 
     * @param entity the entity to add all world forces to.
     * @param increment the percent of world forces to add.
     */
    protected void addWorldForcesTo(Shape entity, double increment) {
        addForceOfGravity(entity, increment);
    }
}
