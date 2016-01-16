package phys2d.collisionLogic.collisionCheckers;

import java.util.ArrayList;

import phys2d.entities.Vec2D;

/**
 * This class holds all the vital information used by the GJK algorithm while it
 * computes whether a collision has taken place.
 * 
 * @author Afsheen
 *
 */
public final class SimplexCollisionInfo extends CollisionInfo {

    /**
     * The simplex for the gjk algorithm.
     */
    protected ArrayList<Vec2D> simplex;

    /**
     * Initialize a new GJKStruct with an empty simplex of size 3 and a search
     * direction = [0,0].
     */
    public SimplexCollisionInfo() {
        this(3);
    }

    public SimplexCollisionInfo(int size) {
        super();
        this.simplex = new ArrayList<Vec2D>(3);
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
     * @return the final simplex after a collision detection algorithm has been
     *         run.
     */
    public ArrayList<Vec2D> getSimplex() {
        return simplex;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#toString()
     */
    @Override
    public String toString() {
        return "SimplexDirStruct [simplex= " + simplex + ", dir= " + dir + ", isColliding= " + isColliding + "]";
    }

}