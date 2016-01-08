/**
 * 
 */
package phys2d.collisionLogic.collisionManagers;

import java.util.ArrayList;
import java.util.HashSet;

import phys2d.Phys2DMain;
import phys2d.collisionLogic.collisionCheckers.CollisionCheckerGJKEPA2;
import phys2d.collisionLogic.collisionCheckers.SimplexDirStruct;
import phys2d.collisionLogic.spacePartitioning.BSPTree;
import phys2d.collisionLogic.spacePartitioning.SweptBSPTree;
import phys2d.collisionLogic.tools.CollisionPair;
import phys2d.collisionLogic.tools.MiscTools;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Rectangle;
import phys2d.entities.shapes.polygons.WorldBound;

/**
 * This class manages all collisions and collision resolutions that take place
 * within the simulation. It uses the 2nd version of the GJK-EPA collision
 * checker and (hopefully) implements some primitive form of swept collision
 * detection. <br>
 * This new class was written because the old manager just got way too messy.
 * The code should now look a lot cleaner and this should run a lot faster.
 * 
 * @author afsheen
 */
public class SpeculativeManager2 extends CollisionManager {

    private final SweptBSPTree collisionTree;

    /**
     * Keeps track of all objects which have been moved for the current
     * timestep.
     */
    private final HashSet<Shape> movedShapes;

    /**
     * Keeps track of all objects which have had world forces applied to them.
     */
    private final HashSet<Shape> forcedShapes;

    /**
     * Keeps track of all pairs of shapes on which collision resolution has been
     * computed.
     */
    private final HashSet<CollisionPair> collidedPairs;

    /**
     * A list of all shape groups which could collide in the current frame.
     */
    private ArrayList<Shape[]> collisionGroups;

    /**
     * Create a new manager which uses GJKv2 and swept detection.
     * 
     * @param dt the timestep of this simulation.
     */
    public SpeculativeManager2(double dt) {
        super(dt);
        collisionTree = new SweptBSPTree(new Rectangle(new Vec2D(500, 500),
                Phys2DMain.XRES + 50, Phys2DMain.YRES + 50),
                BSPTree.HORIZONTAL_SPLIT, 1, dt);

        collidedPairs = new HashSet<CollisionPair>();

        movedShapes = new HashSet<Shape>();
        forcedShapes = new HashSet<Shape>();

    }

    @Override
    protected void manageCollisions(ArrayList<Shape> entities) {
        collisionTree.refresh();

        collidedPairs.clear();
        movedShapes.clear();
        forcedShapes.clear();

        for (Shape s : entities) {
            collisionTree.insert(s);
        }

        collisionGroups = collisionTree.getPossibleCollisions();

        for (Shape[] group : collisionGroups) { // For each group

            /*
             * Perform a "brute force" check. This is now OK because the groups
             * are very small. Also, make sure collision resolution is only run
             * once per shape pair.
             */
            for (int i = 0; i < group.length; i++) {
                for (int j = i + 1; j < group.length; j++) {
                    if (!(group[i] instanceof WorldBound && group[j] instanceof WorldBound)
                            && collidedPairs.add(new CollisionPair(group[i], group[j]))) {
                        resolveCollision(group[i], group[j]);
                    }
                }
            }
        }
    }

    /**
     * 
     * @param s1
     * @param s2
     */
    private void resolveCollision(Shape s1, Shape s2) {
        SimplexDirStruct gjkInfo = CollisionCheckerGJKEPA2.getCollisionResolution(s1, s2);

        if (gjkInfo.isColliding()) { // Discrete collision
            /*
             * Apply world forces
             * unstick
             * add collision forces
             * move
             */
            //System.out.println("disc");

            addWorldForcesTo(s1, 1.0);
            addWorldForcesTo(s2, 1.0);

            forcedShapes.add(s1);
            forcedShapes.add(s2);

            unstickShapes(s1, s2, gjkInfo);
            computeCollisionForces(s1, s2, gjkInfo);

            s1.move(dt);
            s2.move(dt);

            movedShapes.add(s1);
            movedShapes.add(s2);
        }
        else {
            /*
             * apply pre collision world forces
             * move till in contact
             * add collision forces
             * add post collision world forces
             * move for remainder of frame
             */
            double collisionTime = impendingCollisionChecker(s1, s2, gjkInfo);
            if (collisionTime >= 0) { // Impending coll.
                //System.out.println("full swept");

                // Apply pre-collision world forces
                addWorldForcesTo(s1, collisionTime);
                addWorldForcesTo(s2, collisionTime);

                // First, move the shapes till they are just in contact.
                s1.incrementMove(dt, collisionTime);
                s2.incrementMove(dt, collisionTime);

                // Now the shapes should be in contact.

                // Percent of frame left to simulate.
                collisionTime = 1.0 - collisionTime;

                // Add collision forces
                gjkInfo.getDir().negate(); // Expected by the force computer.
                computeCollisionForces(s1, s2, gjkInfo);
                gjkInfo.getDir().negate();

                // Add post-collision world forces.
                addWorldForcesTo(s1, collisionTime);
                addWorldForcesTo(s2, collisionTime);

                forcedShapes.add(s1);
                forcedShapes.add(s2);

                // Now move the shapes by the time left in the frame.
                s1.incrementMove(dt, collisionTime);
                s2.incrementMove(dt, collisionTime);

                movedShapes.add(s1);
                movedShapes.add(s2);
            }
            else { // otherwise, just add all world forces
                addWorldForcesTo(s1, 1.0);
                addWorldForcesTo(s2, 1.0);
                forcedShapes.add(s1);
                forcedShapes.add(s2);
            }

        }
    }

    /**
     * Applies the corresponding forces to the two shapes because of their
     * collision.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param gjkInfo the result of running GJKEPA2 on the two shapes.
     */
    private void computeCollisionForces(Shape s1, Shape s2, SimplexDirStruct gjkInfo) {

        Vec2D unitDisp = gjkInfo.getDir().getNormalized();
        Vec2D relVel = Vec2D.sub(s1.getVelocity(), s2.getVelocity());

        relVel.scaleBy(dt);

        // This is the exact speed along collision axis (projection) because
        // unitDisp is normalized.
        double relNormSp = relVel.dotProduct(unitDisp);

        // This restitution approximation will give pretty believable results.
        double restitution = Math.min(s1.getMaterial().getRestitution(), s2.getMaterial().getRestitution());

        // This formula was made almost a year ago. I'm pretty sure it works.
        Vec2D collForce = Vec2D.getScaled(unitDisp, -(1 + restitution) * relNormSp * (1.0 / dt));

        collForce.scaleBy(1.0 / (s1.getInvMass() + s2.getInvMass()));

        s1.addForce(collForce);
        s2.addForce(collForce.getNegated());

        // calculate friction forces
        relVel = Vec2D.sub(s1.getVelocity(), s2.getVelocity());
        relVel.scaleBy(dt);
        Vec2D tanVec = Vec2D.sub(relVel, relVel.vecProjection(unitDisp));

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
     * Checks to see if there will be a collision between these two shapes in
     * the next frame. If there is, return the exact time during the next frame
     * that the collision should happen.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param gjkInfo the result of the GJKEPA run on the two shapes.
     * @return a double between 0.0-1.0 representing when a collision will take
     *         place next frame. If there is no collision next frame, return -1.
     */
    private double impendingCollisionChecker(Shape s1, Shape s2,
            SimplexDirStruct gjkInfo) {

        Vec2D unitDisp = gjkInfo.getDir().getNormalized();
        Vec2D relVel = Vec2D.sub(s1.getVelocity(), s2.getVelocity());

        relVel.scaleBy(dt);

        // If the shapes are touching, move them back 1 frame to see their
        // previous positions to deduce a collision normal.
        if (unitDisp.equals(Vec2D.ORIGIN)) {
            for (Vec2D v : gjkInfo.getSimplex()) {
                v.sub(relVel);
            }
            computeMinimumDisplacement(gjkInfo);
            unitDisp = gjkInfo.getDir().getNormalized();
        }

        // The speed along the collision normal
        double relNormSpeed = relVel.dotProduct(unitDisp);
        double seperatingDist = gjkInfo.getDir().getLength();

        if (relNormSpeed >= seperatingDist) {
            return seperatingDist / relNormSpeed;
        }

        return -1;
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

    private void computeMinimumDisplacement(SimplexDirStruct gjkInfo) {
        if (gjkInfo.getSimplex().size() == 1)
            gjkInfo.setDir(gjkInfo.getSimplex().get(0));

        else {
            Vec2D AB = Vec2D.sub(gjkInfo.getSimplex().get(0),
                    gjkInfo.getSimplex().get(1));
            Vec2D AO = gjkInfo.getSimplex().get(1).getNegated();

            gjkInfo.setDir(AO.vecProjection(AB));
        }

    }

    /**
     * If a collision is detected, translate the shapes out of each other. <br>
     * The translation is distributed between each shape depending on mass. This
     * should be very small anyways because deep collisions will be prevented by
     * the swept checker.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param gjkInfo the result of the GJKEPA run on the two shapes.
     */
    private void unstickShapes(Shape s1, Shape s2, SimplexDirStruct gjkInfo) {

        double totalMass = s1.getMass() + s2.getMass();
        double s1ratio, s2ratio;

        // Logically distribute the translations.
        if (s1 instanceof WorldBound) {
            s1ratio = 0;
            s2ratio = 1;
        }
        else if (s2 instanceof WorldBound) {
            s1ratio = 1;
            s2ratio = 0;
        }
        else {
            s1ratio = s1.getMass() / totalMass;
            s2ratio = s2.getMass() / totalMass;
        }

        Vec2D s1tran, s2tran;

        s1tran = gjkInfo.getDir().getNegated().getScaled(s1ratio);
        s2tran = gjkInfo.getDir().getScaled(s2ratio);

        s1.translate(s1tran);
        s2.translate(s2tran);

        // System.out.println("Shapes unstuck! " + gjkInfo.getDir());

    }

    @Override
    public void runManager(ArrayList<Shape> entities) {

        manageCollisions(entities);

        addWorldForces(entities);

        moveEntities(entities);

    }

    @Override
    protected void moveEntities(ArrayList<Shape> entities) {
        for (Shape s : entities) {
            if (movedShapes.add(s)) {
                s.move(dt);
            }
        }
    }

    @Override
    protected void addWorldForces(ArrayList<Shape> entities) {
        for (Shape s : entities) {
            if (!forcedShapes.contains(s)) {
                addWorldForcesTo(s, 1.0);
                forcedShapes.add(s);
            }
        }
    }

    @Override
    protected void addWorldForcesTo(Shape entity, double increment) {
        if (!forcedShapes.contains(entity)) {
            addForceOfGravity(entity, increment);
        }
    }

    @Override
    public BSPTree getCollisionTree() {
        return collisionTree;
    }

}
