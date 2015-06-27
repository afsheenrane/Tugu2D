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
 * checker and (hopefully) implements speculative contacts.
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

        for (Shape[] group : collisionGroups) {

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
            // TODO unstick shapes and compute forces
            unstickShapes(s1, s2, gjkInfo);
            // computeForces(s1, s2, gjkInfo);
        }
        else { // No discrete collision
               // TODO check if collision imminent

            // If imminent, execute speculative contacts algo

            // otherwise, no one cares
        }

    }

    private void unstickShapes(Shape s1, Shape s2, SimplexDirStruct gjkInfo) {
        double totalMass = s1.getMass() + s2.getMass();
        double s1ratio, s2ratio;

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

        System.out.println("Shapes unstuck! " + gjkInfo.getDir());

    }

    @Override
    public void runManager(ArrayList<Shape> entities) {
        // addWorldForces(entities);

        manageCollisions(entities);

        moveEntities(entities);

    }

}
