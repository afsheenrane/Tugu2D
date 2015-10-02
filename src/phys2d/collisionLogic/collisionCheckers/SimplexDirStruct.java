package phys2d.collisionLogic.collisionCheckers;

import java.util.ArrayList;

import phys2d.entities.Vec2D;

/**
 * This class holds all the vital information used by any collision detection
 * algorithm which computes a CSO simplex, while it computes whether a collision
 * has taken place.
 * 
 * @author Afsheen
 *
 */
public final class SimplexDirStruct {

    /**
     * The simplex for the gjk/mpr algorithm.
     */
    protected ArrayList<Vec2D> simplex;

    /**
     * Stores which Shape vertices created which simplex points. <br>
     * The indices of this list are directly mapped to the simplex list indices.
     * The indices of the inner array correspond to s1 and s2.
     */
    protected ArrayList<Vec2D[]> corrShapePts;

    /**
     * The current search direction.
     */
    protected Vec2D dir;

    protected boolean isColliding;

    /**
     * Initialize a new SimplexDirStruct with an empty simplex of size 3 and a
     * search direction = [0,0].
     */
    protected SimplexDirStruct() {
        this(3);
    }

    protected SimplexDirStruct(int size) {
        this.simplex = new ArrayList<Vec2D>(size);
        this.corrShapePts = new ArrayList<Vec2D[]>(3);
        this.dir = new Vec2D();
        this.isColliding = false;
    }

    /**
     * @return the final simplex after a collision detection algorithm has been
     *         run.
     */
    public ArrayList<Vec2D> getSimplex() {
        return simplex;
    }

    /**
     * @return the collision normal or displacement normal between the two
     *         shapes on which a collision detection algorithm is run. <br>
     *         <b>Note: </b> <i>Only use this getter outside of the actual
     *         collision detection computation. This is because, dir is only a
     *         collision normal AFTER a collision detection algorithm has been
     *         executed. Otherwise, directly access dir during computation.</i>
     */
    public Vec2D getDir() {
        return dir;
    }

    /**
     * Sets the last search direction for this structure.
     * This is only to be used outside a collision detection algorithm. Careful.
     * 
     * @param dir the dir to set.
     */
    public void setDir(Vec2D dir) {
        this.dir = dir;
    }

    /**
     * @return whether the shapes to which this struct belonged to were
     *         colliding or not.
     */
    public boolean isColliding() {
        return isColliding;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#toString()
     */
    @Override
    public String toString() {
        return "SimplexDirStruct [simplex= " + simplex + ", dir= " + dir
                + ", isColliding= " + isColliding + "]";
    }

}