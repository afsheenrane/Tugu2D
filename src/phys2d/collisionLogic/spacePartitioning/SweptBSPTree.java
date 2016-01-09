package phys2d.collisionLogic.spacePartitioning;

import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;

public class SweptBSPTree extends BSPTree {

    private final double dt;

    /**
     * Create a new SweptBSPTree with the given bounds, splitmode, the current
     * level of the node, and the current physics delta time (to compute AABB's
     * of the shapes).
     * 
     * @param bounds the bounding rectangle that this node covers in the game
     *            world, <b>in MIN-MAX notation<b>.
     * @param splitMode the current split mode of this node.
     * @param level the current level of this node.
     * @param dt the physics delta time of the simulation.
     */
    public SweptBSPTree(Vec2D[] bounds, int splitMode, int level, double dt) {
        super(bounds, splitMode, level);
        this.dt = dt;
    }

    @Override
    protected int getInsertionSide(Shape s) {
        Vec2D[] sAabb = s.getSweptAABBbounds(dt);
        return getInsertionSideAABB(sAabb);
    }

    @Override
    public void split() {
        SweptBSPTree c0, c1; // child 1, child 2

        Vec2D[] newBounds = new Vec2D[2];

        if (splitMode == VERTICAL_SPLIT) {

            double midX = (bounds[0].getX() + bounds[1].getX()) / 2.0;

            newBounds[0] = bounds[0];
            newBounds[1] = new Vec2D(midX, bounds[1].getY());
            c0 = new SweptBSPTree(newBounds.clone(), splitMode * -1, depth + 1, dt); // left rect

            newBounds[0] = new Vec2D(midX, bounds[0].getY());
            newBounds[1] = bounds[1];
            c1 = new SweptBSPTree(newBounds.clone(), splitMode * -1, depth + 1, dt); // right rect
        }
        else { //HORIZONTAL_SPLIT

            double midY = (bounds[0].getY() + bounds[1].getY()) / 2.0;

            newBounds[0] = bounds[0];
            newBounds[1] = new Vec2D(bounds[1].getX(), midY);
            c0 = new SweptBSPTree(newBounds.clone(), splitMode * -1, depth + 1, dt); // bottom (top in GUI)

            newBounds[0] = new Vec2D(bounds[0].getX(), midY);
            newBounds[1] = bounds[1];
            c1 = new SweptBSPTree(newBounds.clone(), splitMode * -1, depth + 1, dt); // top (bottom in gui)
        }

        children[0] = c0;
        children[1] = c1;

    }

}
