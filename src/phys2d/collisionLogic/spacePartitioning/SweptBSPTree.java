package phys2d.collisionLogic.spacePartitioning;

import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Rectangle;

public class SweptBSPTree extends BSPTree {

    private final double dt;

    public SweptBSPTree(Rectangle bounds, int splitMode, int level, double dt) {
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

        Rectangle newBounds;
        Vec2D com = bounds.getCOM().getCopy();

        if (splitMode == VERTICAL_SPLIT) {

            com.setX(com.getX() - (bounds.getLength() / 4));
            newBounds = new Rectangle(com.getCopy(), bounds.getLength() / 2,
                    bounds.getHeight()); // doing com copy because com is being aliased. not copying newbounds because it is being being re-assigned
            c0 = new SweptBSPTree(newBounds, splitMode * -1, depth + 1, dt); // left rect

            com.setX(com.getX() + bounds.getLength() / 2);
            newBounds = new Rectangle(com.getCopy(), bounds.getLength() / 2,
                    bounds.getHeight());
            c1 = new SweptBSPTree(newBounds, splitMode * -1, depth + 1, dt); // right rect
        }
        else {

            com.setY(com.getY() - (bounds.getHeight() / 4));
            newBounds = new Rectangle(com.getCopy(), bounds.getLength(),
                    bounds.getHeight() / 2);
            c0 = new SweptBSPTree(newBounds, splitMode * -1, depth + 1, dt); // bottom (top in GUI)

            com.setY(com.getY() + bounds.getHeight() / 2);
            newBounds = new Rectangle(com.getCopy(), bounds.getLength(),
                    bounds.getHeight() / 2);
            c1 = new SweptBSPTree(newBounds, splitMode * -1, depth + 1, dt); // top (bottom in gui)
        }

        children[0] = c0;
        children[1] = c1;

    }

}
