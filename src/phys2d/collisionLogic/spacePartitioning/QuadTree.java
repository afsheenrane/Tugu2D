package phys2d.collisionLogic.spacePartitioning;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.List;

import phys2d.Phys2DMain;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;

public class QuadTree extends SpacePartitioningTree {

    protected int depth = 0;

    protected List<Shape> items;
    protected final Vec2D[] bounds;
    /**
     * Children are ordered clockwise from the bottom right.
     */
    protected QuadTree[] children;

    /**
     * Create a new QuadTree with the given bounds and the current level of this
     * node.
     * 
     * @param bounds the bounding rectangle that this node covers in the game
     *            world, <b>in MIN-MAX notation<b>.
     * @param level the current level of this node.
     */
    public QuadTree(Vec2D[] bounds, int level) {
        super(5, 4);
        this.bounds = bounds;
        this.depth = level;

        items = new ArrayList<Shape>();
        children = new QuadTree[4];
    }

    // http://gamedevelopment.tutsplus.com/tutorials/quick-tip-use-quadtrees-to-detect-likely-collisions-in-2d-space--gamedev-374
    /**
     * Splits the current node into four separate children.
     */
    protected void split() {

        Vec2D center = new Vec2D((bounds[0].getX() + bounds[1].getX()) / 2.0,
                (bounds[0].getY() + bounds[1].getY()) / 2.0);

        children[0] = new QuadTree(new Vec2D[] { bounds[0].getCopy(), center.getCopy() }, depth + 1); //BL

        children[1] = new QuadTree(new Vec2D[] { //TL
                new Vec2D(bounds[0].getX(), center.getY()),
                new Vec2D(center.getX(), bounds[1].getY())
        }, depth + 1);

        children[2] = new QuadTree(new Vec2D[] { center.getCopy(), bounds[1].getCopy() }, depth + 1); //TR

        children[3] = new QuadTree(new Vec2D[] { //BR
                new Vec2D(center.getX(), bounds[0].getY()),
                new Vec2D(bounds[1].getX(), center.getY())
        }, depth + 1);
    }

    /**
     * Returns all the children that the entered aabb fits in.
     * 
     * @param aabb the axis aligned bounding box to check (probably of a shape).
     * @return an encoded byte containing all the children that the aabb can fit
     *         into. The lowest order bit is for child0, second lowest is
     *         child1, and so forth.
     */
    protected byte getInsertionSide(Vec2D[] aabb) {
        byte retByte = 0;
        byte curChild = 1;

        for (int i = 0; i < children.length; i++) {
            if (childCanContainAABB(children[i], aabb)) {
                retByte |= curChild;
            }
            curChild <<= 1;
        }

        return retByte;

    }

    /**
     * Get which sides the Shape s can be inserted into.
     * 
     * @param s the shape to check.
     * @return an encoded byte containing all the children that the shape can
     *         fit into. The lowest order bit is for child0, second lowest is
     *         child1, and so forth.
     */
    protected byte getInsertionSide(Shape s) {
        return getInsertionSide(s.getAABBbounds());
    }

    /**
     * Checks to see whether the entered aabb intersects with the bounds of the
     * entered child.
     * 
     * @param child the QuadTree to check the aabb against.
     * @param aabb the aabb of the shape to check.
     * @return true if the aabb intersects with the child quadtree.
     */
    protected boolean childCanContainAABB(QuadTree child, Vec2D[] aabb) {
        //TODO, should eventually move this to the SAT collision checker.
        return !(child.bounds[0].getX() > aabb[1].getX() || child.bounds[1].getX() < aabb[0].getX() ||
                child.bounds[0].getY() > aabb[1].getY() || child.bounds[1].getY() < aabb[0].getY());

    }

    /**
     * Insert the shape into the current node of the quadtree.
     * 
     * @param s the shape to insert.
     */
    @Override
    public void insert(Shape s) {

        // If there are children to add into
        if (children[0] != null) {
            insertShapeIntoChildren(s);
            return;
        }

        //If there are no children, try adding to parent.
        items.add(s);

        // But, now if we have overloaded this node, we need to split it down some more.
        // Also, we can only split if we havent exceeded the level_cap (depth cap).

        if (items.size() > MAX_ITEMS && depth < DEPTH_CAP) {
            split();

            //Now that children exist, offload all the shapes into them.
            for (Shape curShape : items) {
                insertShapeIntoChildren(curShape);
            }
            items.clear();
        }
    }

    /**
     * Inserts the shape s into all possible children.
     * 
     * @param s the shape to insert.
     * @param sAABB the shape's axis aligned bounding box. <br>
     *            (Mainly here for efficiency reasons. To prevent redundant
     *            recalculation of the AABB).
     */
    protected void insertShapeIntoChildren(Shape s) {
        Vec2D[] aabb = s.getAABBbounds();
        for (int i = 0; i < children.length; i++) {
            if (childCanContainAABB(children[i], aabb)) {
                children[i].insert(s);
            }
        }
    }

    /**
     * Gets a list of shapes which have been grouped with other shapes which
     * they might collide with.
     * 
     * @return an arraylist of all shape groups that need to be checked in the
     *         narrowphase.
     */
    @Override
    public ArrayList<Shape[]> getPossibleCollisions() {
        ArrayList<Shape[]> collisionGroups = new ArrayList<Shape[]>(50);
        computeCollisionGroups(collisionGroups);
        return collisionGroups;
    }

    /**
     * Recursively visits each node of the quadtree and groups all shapes that
     * occur in the same node.
     * 
     * @param collisionGroups the list containing the collision groups.
     */
    protected void computeCollisionGroups(ArrayList<Shape[]> collisionGroups) {
        //If this node has no children
        if (children[0] == null) {
            if (items.size() > 1) //If there is more that 1 item in this node, make a group.
                collisionGroups.add(items.toArray(new Shape[items.size()]));
        }
        else {
            children[0].computeCollisionGroups(collisionGroups);
            children[1].computeCollisionGroups(collisionGroups);
            children[2].computeCollisionGroups(collisionGroups);
            children[3].computeCollisionGroups(collisionGroups);
        }
    }

    /**
     * Cleans this tree by reseting it's children and items.
     */
    @Override
    public void refresh() {
        items.clear();
        children = new QuadTree[4];
    }

    @Override
    public void draw(Graphics2D g2d) {
        Color t = g2d.getColor();
        g2d.setColor(Color.ORANGE);

        Vec2D com = new Vec2D((bounds[0].getX() + bounds[1].getX()) / 2.0, (bounds[0].getY() + bounds[1].getY()) / 2.0);

        g2d.drawString(depth + " " + items.size(), (int) com.getX() - 10, Phys2DMain.YRES - (int) (com.getY() + 5));
        if (children[0] != null) {
            g2d.drawLine((int) com.getX(), Phys2DMain.YRES - (int) bounds[0].getY(), (int) com.getX(),
                    Phys2DMain.YRES - (int) bounds[1].getY());
            g2d.drawLine((int) bounds[0].getX(), Phys2DMain.YRES - (int) com.getY(), (int) bounds[1].getX(),
                    Phys2DMain.YRES - (int) com.getY());

            for (QuadTree child : children) {
                child.draw(g2d);
            }
        }

        g2d.setColor(t);
    }

}
