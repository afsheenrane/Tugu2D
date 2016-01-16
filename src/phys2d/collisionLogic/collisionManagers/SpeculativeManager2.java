/**
 * 
 */
package phys2d.collisionLogic.collisionManagers;

import java.util.ArrayList;
import java.util.HashSet;

import phys2d.Phys2DMain;
import phys2d.collisionLogic.collisionCheckers.CollisionChecker;
import phys2d.collisionLogic.collisionCheckers.CollisionCheckerGJKEPA2;
import phys2d.collisionLogic.collisionCheckers.CollisionInfo;
import phys2d.collisionLogic.collisionCheckers.SweptCapable;
import phys2d.collisionLogic.spacePartitioning.SpacePartitioningTree;
import phys2d.collisionLogic.spacePartitioning.SweptQuadTree;
import phys2d.collisionLogic.tools.CollisionPair;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
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

    private final SpacePartitioningTree collisionTree;

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
     * The structure that holds all the info of each collision.
     */
    private CollisionInfo collisionInfo;

    private final CollisionChecker collisionSolver;

    /**
     * Flag to check if the solver is capable of swept detection.
     */
    private final boolean isSweptCapable;

    /**
     * Practically a copy of the collisionSolver. This just exists to prevent
     * having to cast the solver into SweptCapable every iteration.
     */
    private SweptCapable sweptSolver;

    /**
     * Create a new manager which uses GJKv2 and swept detection.
     * 
     * @param dt the timestep of this simulation.
     */
    public SpeculativeManager2(double dt) {
        this(dt, new CollisionCheckerGJKEPA2(), false);
    }

    /**
     * Create a new swept capable solver using the collisionSolver provided.
     * <br>
     * Note: If the entered collisionSolver is not <code>SweptCapable</code>
     * this manager will default to discrete collision detection.
     * 
     * @param dt the timestep of this simulation.
     * @param collisionSolver the algorithm set which will be used to compute
     *            collisions.
     * @param forceDiscreteSolver if true, this manager will only run the
     *            simulation using discrete timestep collision resolution.
     *            <br>
     *            Has no effect if the CollisionSolver is not swept capable.
     */
    public SpeculativeManager2(double dt, CollisionChecker collisionSolver, boolean forceDiscreteSolver) {
        super(dt);
        collisionTree = new SweptQuadTree(new Vec2D[] { new Vec2D(-10, -10),
                new Vec2D(Phys2DMain.XRES + 10, Phys2DMain.YRES + 10) }, 1, dt);

        //collisionTree = new SweptBSPTree(new Vec2D[] { new Vec2D(-10, -10),
        //       new Vec2D(Phys2DMain.XRES + 10, Phys2DMain.YRES + 10) }, BSPTree.HORIZONTAL_SPLIT, 1, dt);

        collidedPairs = new HashSet<CollisionPair>();

        this.collisionSolver = collisionSolver;

        this.isSweptCapable = forceDiscreteSolver ? false : this.collisionSolver instanceof SweptCapable;

        if (isSweptCapable) {
            sweptSolver = (SweptCapable) this.collisionSolver;
        }
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

        // The following is just to simulate full brute force without space partitioning
        //        for (int i = 0; i < entities.size(); i++) {
        //            for (int j = i + 1; j < entities.size(); j++) {
        //                if (!(entities.get(i) instanceof WorldBound && entities.get(j) instanceof WorldBound)
        //                        && collidedPairs.add(new CollisionPair(entities.get(i), entities.get(j)))) {
        //                    resolveCollision(entities.get(i), entities.get(j));
        //                }
        //            }
        //        }

    }

    /**
     * 
     * @param s1
     * @param s2
     */
    private void resolveCollision(Shape s1, Shape s2) {
        collisionInfo = collisionSolver.getCollisionResolution(s1, s2);

        if (collisionInfo.isColliding()) { // Discrete collision
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

            unstickShapes(s1, s2, collisionInfo.getDir());
            applyCollisionForces(s1, s2, collisionInfo.getDir());

            s1.move(dt);
            s2.move(dt);

            movedShapes.add(s1);
            movedShapes.add(s2);
        }
        else if (isSweptCapable) {
            /*
             * apply pre collision world forces
             * move till in contact
             * add collision forces
             * add post collision world forces
             * move for remainder of frame
             */
            double collisionTime = sweptSolver.getImpendingCollisionTime(s1, s2, collisionInfo, dt);
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
                //gjkInfo.getDir().negate(); // Expected by the force computer.
                applyCollisionForces(s1, s2, collisionInfo.getDir());
                //gjkInfo.getDir().negate();

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
     * If a collision is detected, translate the shapes out of each other. <br>
     * The translation is distributed between each shape depending on mass. This
     * should be very small anyways because deep collisions will be prevented by
     * the swept checker.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param collisionAxis the axis (and direction) along which s2 needs to be
     *            translated by,to be completely removed from s1.
     */
    private void unstickShapes(Shape s1, Shape s2, Vec2D collisionAxis) {

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

        s1tran = collisionAxis.getNegated().getScaled(s1ratio);
        s2tran = collisionAxis.getScaled(s2ratio);

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

    /**
     * Return the collision tree being used to partition the entities.
     * 
     * @return the space partitioning binary tree.
     */
    public SpacePartitioningTree getCollisionTree() {
        return collisionTree;
    }

}
