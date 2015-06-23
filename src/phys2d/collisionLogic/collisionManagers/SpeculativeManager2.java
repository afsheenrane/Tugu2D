/**
 * 
 */
package phys2d.collisionLogic.collisionManagers;

import java.util.ArrayList;

import phys2d.entities.shapes.Shape;

/**
 * This class manages all collisions and collision resolutions that take
 * place within the simulation. It uses the 2nd version of the GJK-EPA
 * collision checker and (hopefully) implements speculative contacts.
 * 
 * @author afsheen
 */
public class SpeculativeManager2 extends CollisionManager {

    /**
     * @param dt
     */
    public SpeculativeManager2(double dt) {
        super(dt);
        // TODO Auto-generated constructor stub
    }

    @Override
    protected void manageCollisions(Shape[] entities) {
        // TODO Auto-generated method stub

    }

    /*
     * (non-Javadoc)
     * 
     * @see
     * phys2d.collisionLogic.collisionManagers.CollisionManager#runManager(java
     * .util.ArrayList)
     */
    @Override
    public void runManager(ArrayList<Shape> entities) {
        // TODO Auto-generated method stub

    }

}
