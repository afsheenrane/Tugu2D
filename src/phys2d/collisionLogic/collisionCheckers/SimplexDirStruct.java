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
public final class SimplexDirStruct {

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

    /**
     * Be very careful when editing this parameters outside the collision
     * checkers.
     * 
     * @param simplex the simplex to set
     */
    public void setSimplex(ArrayList<Vec2D> simplex) {
        this.simplex = simplex;
    }

    /**
     * Be very careful when editing this parameters outside the collision
     * checkers.
     * 
     * @param dir the dir to set
     */
    public void setDir(Vec2D dir) {
        this.dir = dir;
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