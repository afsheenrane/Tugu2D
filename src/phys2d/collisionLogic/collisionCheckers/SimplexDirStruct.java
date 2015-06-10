package phys2d.collisionLogic.collisionCheckers;

import java.util.ArrayList;

import phys2d.entities.Vec2D;

/**
 * This class holds all the vital information used by th GJK algorithm while it
 * computes whether a collision has taken place.
 * 
 * @author Afsheen
 *
 */
final class SimplexDirStruct {

    /**
     * The simplex for the gjk algorithm.
     */
    protected ArrayList<Vec2D> simplex;

    /**
     * The current search direction.
     */
    protected Vec2D dir;

    protected boolean isColliding;

    /**
     * Initialize a new GJKStruct with an empty simplex of size 3 and a search
     * direction = [0,0].
     */
    protected SimplexDirStruct() {
        this(3);
    }

    protected SimplexDirStruct(int size) {
        this.simplex = new ArrayList<Vec2D>(3);
        this.dir = new Vec2D();
        this.isColliding = false;
    }

}