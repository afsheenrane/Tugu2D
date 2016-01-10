/**
 * 
 */
package phys2d.collisionLogic.collisionManagers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

import phys2d.collisionLogic.tools.MiscTools;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.WorldBound;

/**
 * @author afsheen
 *
 */
public abstract class CollisionManager {

    /**
     * Keeps track of all objects which have been moved for the current
     * timestep.
     */
    protected final HashSet<Shape> movedShapes;

    /**
     * Keeps track of all objects which have had world forces applied to them.
     */
    protected final HashSet<Shape> forcedShapes;

    protected final double dt;

    protected double g = -100.0; // m/s^2

    public CollisionManager(double dt) {
        this.dt = dt;
        movedShapes = new HashSet<Shape>();
        forcedShapes = new HashSet<Shape>();
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

    /**
     * Sets the force of gravity in the simulation to g.
     * 
     * @param g the force of gravity to set.
     */
    public void setForceOfGravity(double g) {
        this.g = g;
    }

    /**
     * Applies the corresponding forces to the two shapes because of their
     * collision.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param collisionAxis a vector that represents the collision axis along
     *            which the collision takes place.
     */
    protected void applyCollisionForces(Shape s1, Shape s2, Vec2D collisionAxis) {

        Vec2D unitCollAxis = collisionAxis.getNormalized();
        Vec2D relVel = Vec2D.sub(s1.getVelocity(), s2.getVelocity());

        relVel.scaleBy(dt);

        // This is the exact speed along collision axis (projection) because unitDisp is normalized.
        double relNormSp = relVel.dotProduct(unitCollAxis);

        // This restitution approximation will give pretty believable results.
        double restitution = Math.min(s1.getMaterial().getRestitution(), s2.getMaterial().getRestitution());

        // This formula was made almost a year ago. I'm pretty sure it works.
        Vec2D collForce = Vec2D.getScaled(unitCollAxis, -(1 + restitution) * relNormSp * (1.0 / dt));

        collForce.scaleBy(1.0 / (s1.getInvMass() + s2.getInvMass()));

        s1.addForce(collForce);
        s2.addForce(collForce.getNegated());

        // calculate friction forces
        relVel = Vec2D.sub(s1.getVelocity(), s2.getVelocity());
        relVel.scaleBy(dt);
        Vec2D tanVec = Vec2D.sub(relVel, relVel.vecProjection(unitCollAxis));

        if (!tanVec.equals(Vec2D.ORIGIN))
            tanVec.normalize();

        double fricMag = -Vec2D.dotProduct(relVel, tanVec); //Because opposite direction

        fricMag /= s1.getInvMass() + s2.getInvMass();

        // Approximate mu using pythagorean theorem
        double mu = Math.sqrt(Math.pow(s1.getMaterial().getStaticFric(), 2)
                + Math.pow(s2.getMaterial().getStaticFric(), 2));

        Vec2D frictionForce = Vec2D.ORIGIN;

        if (Math.abs(fricMag) < collForce.getLength() * mu) {
            if (!MiscTools.tolEquals(-fricMag, 0, 1e-6))
                frictionForce = tanVec.getScaled(-fricMag);
        }
        else {
            mu = Math.sqrt(Math.pow(s1.getMaterial().getDynFric(), 2) + Math.pow(s2.getMaterial().getDynFric(), 2));

            if (!MiscTools.tolEquals(-fricMag * mu, 0, 1e-6))
                frictionForce = tanVec.getScaled(-fricMag * mu);

        }

        // The following is a "kill all velocity" routine for when static friction is active and 
        // an object is moving very slowly. This should stop objects creeping.

        Vec2D velocityKillValue = getKillVelocityValue(s1, tanVec);
        if (velocityKillValue != Vec2D.ORIGIN) {

            //Negate it, because it represents the tangential velocity, not the friction direction
            velocityKillValue.negate();
            velocityKillValue.scaleBy(s1.getMass()); //Yeah yeah, it's a force now. Sue me on the naming scheme.

            s1.addForce(velocityKillValue);

            /*
             * Check for any floating point error inside the shape's velocity.
             * The only way to kill tangential velocity in the \general\ case is to apply a force along the tangential vector.
             * We can't just 0 out the velocity along the x or y component. 
             * But very often, 0ing out the x/y component of velocity, IS the correct step. The force application itself might fail 
             * by a miniscule amount (<1e-15), therefore, we must manually 0 out the component. 
             */
            s1.incrementMove(dt, 1);

            if (MiscTools.tolEquals(s1.getVelocity().getX(), 0)) {
                s1.getVelocity().setX(0);
            }
            if (MiscTools.tolEquals(s1.getVelocity().getY(), 0)) {
                s1.getVelocity().setY(0);
            }

            movedShapes.add(s1);
        }
        else { //Friction value high enough, no problemo!
            s1.addForce(frictionForce.getNegated());
        }

        //Because we're checking ALONG the shape's velocity vector, not opposite to it.
        velocityKillValue = getKillVelocityValue(s2, tanVec.getNegated());

        if (velocityKillValue != Vec2D.ORIGIN) {
            velocityKillValue.negate();
            velocityKillValue.scaleBy(s2.getMass());

            s2.addForce(velocityKillValue);

            s2.incrementMove(dt, 1);

            if (MiscTools.tolEquals(s2.getVelocity().getX(), 0)) {
                s2.getVelocity().setX(0);
            }
            if (MiscTools.tolEquals(s2.getVelocity().getY(), 0)) {
                s2.getVelocity().setY(0);
            }

            movedShapes.add(s2);
        }
        else { //Friction value high enough, no problemo!
            s2.addForce(frictionForce);
        }

        s1.incrementMove(dt, 0);
        s2.incrementMove(dt, 0);
    }

    /**
     * Advises whether the shape should be stationary along the given friction
     * vector.
     * 
     * @param s the shape to check.
     * @param tanVec the tangential vector along which friction is to be
     *            applied.
     * @return whether the Shape is moving very slowly along it's friction
     *         vector. True when there is very little movement and the shape
     *         should really just be stationary.
     */
    private Vec2D getKillVelocityValue(Shape s, Vec2D tanVec) {

        if (s instanceof WorldBound)
            return Vec2D.ORIGIN;

        final double SPEED_TOL = 0.5; //Magic number that works all right

        Vec2D tanVel = s.getVelocity().vecProjection(tanVec);

        if (tanVel.getSquaredLength() <= SPEED_TOL * SPEED_TOL) {
            return tanVel;
        }

        return Vec2D.ORIGIN;
    }

}
