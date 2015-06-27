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
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Rectangle;
import phys2d.entities.shapes.polygons.WorldBound;

/**
 * This class manages all collisions and collision resolutions that take place
 * within the simulation. It uses the 2nd version of the GJK-EPA collision
 * checker and (hopefully) implements speculative contacts. <br>
 * This new class was written because the old manager just got way too messy.
 * The code should now look a lot cleaner and this should run a lot faster.
 * 
 * @author afsheen
 */
public class SpeculativeManager2 extends CollisionManager {

    private final SweptBSPTree collisionTree;

    /**
     * Create a new manager which uses GJKv2 and speculative contacts.
     * 
     * @param dt the timestep of this simulation.
     */
    public SpeculativeManager2(double dt) {
        super(dt);
        collisionTree = new SweptBSPTree(new Rectangle(new Vec2D(500, 500),
                Phys2DMain.XRES + 50, Phys2DMain.YRES + 50),
                BSPTree.HORIZONTAL_SPLIT, 1, dt);
    }

    @Override
    protected void manageCollisions(ArrayList<Shape> entities) {
        collisionTree.refresh();
        HashSet<CollisionPair> collidedPairs = new HashSet<CollisionPair>(
                entities.size());

        for (Shape s : entities) {
            collisionTree.insert(s);
        }

        ArrayList<Shape[]> collisionGroups = collisionTree
                .getPossibleCollisions();

        for (Shape[] group : collisionGroups) { // For each group

            /*
             * Perform a "brute force" check. This is now OK because the groups
             * are very small. Also, make sure collision resolution is only run
             * once per shape pair.
             */
            for (int i = 0; i < group.length; i++) {
                for (int j = i + 1; j < group.length; j++) {
                    if (!(group[i] instanceof WorldBound && group[j] instanceof WorldBound)
                            && collidedPairs.add(new CollisionPair(group[i],
                                    group[j]))) {
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
        SimplexDirStruct gjkInfo = CollisionCheckerGJKEPA2
                .getCollisionResolution(s1, s2);

        if (gjkInfo.isColliding()) {
            unstickShapes(s1, s2, gjkInfo);
            computeForces(s1, s2, gjkInfo);
            return;
        }
        else { // No discrete collision
            if (isCollisionImminent(s1, s2, gjkInfo)) {
                // If imminent, execute speculative contacts algo
            }
            // otherwise, no one cares
        }
    }

    /**
     * Checks to see if there will be a collision between these two shapes in
     * the next frame.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param gjkInfo the result of the GJKEPA run on the two shapes.
     * @return true if there is a collision imminent next frame. False
     *         otherwise.
     */
    private boolean isCollisionImminent(Shape s1, Shape s2,
            SimplexDirStruct gjkInfo) {
        // TODO Auto-generated method stub
        return false;
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
            s1ratio = 0;
            s2ratio = 1;
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

    /**
     * Applies the corresponding forces to the two shapes because of their
     * collision.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param gjkInfo the result of running GJKEPA2 on the two shapes.
     */
    private void computeForces(Shape s1, Shape s2, SimplexDirStruct gjkInfo) {

        Vec2D unitDisp = gjkInfo.getDir().getNormalized();
        Vec2D relVel = Vec2D.sub(s1.getVelocity(), s2.getVelocity());

        relVel.scaleBy(dt);

        // This is the exact speed along collision axis (projection) because
        // unitDisp is normalized.
        double relNormSp = relVel.dotProduct(unitDisp);

        // This restitution approximation will give pretty believable results.
        double restitution = Math.min(s1.getMaterial().getRestitution(), s2
                .getMaterial().getRestitution());

        // This formula was made almost a year ago. I'm pretty sure it works.
        Vec2D force = Vec2D.getScaled(unitDisp, -(1 + restitution) * relNormSp
                * (1.0 / dt));

        force.scaleBy(1.0 / (s1.getInvMass() + s2.getInvMass()));

        s1.addForce(force);
        s2.addForce(force.getNegated());

    }

    @Override
    public void runManager(ArrayList<Shape> entities) {
        // addWorldForces(entities);

        manageCollisions(entities);

        moveEntities(entities);

    }

}
