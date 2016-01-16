package phys2d.collisionLogic.collisionCheckers;

import phys2d.entities.Vec2D;

public class CollisionInfo {

    /**
     * The current search direction.
     */
    protected Vec2D dir;

    protected boolean isColliding;

    public CollisionInfo() {
        this.dir = new Vec2D();
        this.isColliding = false;
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
        return "CollisionInfo [dir= " + dir + ", isColliding= " + isColliding + "]";
    }

}
