/**
 * 
 */
package phys2d.collisionLogic.collisionManagers;

import java.util.ArrayList;
import java.util.HashSet;

import phys2d.Phys2DMain;
import phys2d.collisionLogic.spacePartitioning.BSPTree;
import phys2d.collisionLogic.spacePartitioning.SweptBSPTree;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Rectangle;

/**
 * This class manages all collisions and collision resolutions that take place
 * within the simulation. It uses the 2nd version of the GJK-EPA collision
 * checker and (hopefully) implements speculative contacts.
 * 
 * @author afsheen
 */
public class SpeculativeManager2 extends CollisionManager {

    private SweptBSPTree collisionTree;

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
        HashSet<CollisionPairs> collidedPairs = new HashSet<CollisionPairs>(
                entities.size());

        for (Shape s : entities) {
            collisionTree.insert(s);
        }

    }

    @Override
    public void runManager(ArrayList<Shape> entities) {
        // addWorldForces(entities);

        manageCollisions(entities);

        moveEntities(entities);

    }

}
