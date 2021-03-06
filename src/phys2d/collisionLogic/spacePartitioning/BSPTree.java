package phys2d.collisionLogic.spacePartitioning;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.List;

import phys2d.Phys2DMain;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.WorldBound;

public class BSPTree extends SpacePartitioningTree {

    public static final int HORIZONTAL_SPLIT = -1;
    public static final int VERTICAL_SPLIT = 1;

    protected int depth = 0;

    protected List<Shape> items;
    protected final Vec2D[] bounds;
    protected BSPTree[] children;
    protected final int splitMode;

    /**
     * Create a new BSPTree with the given bounds, splitmode of horizonal or
     * vertical and the current level of this node.
     * 
     * @param bounds the bounding rectangle that this node covers in the game
     *            world, <b>in MIN-MAX notation<b>.
     * @param splitMode the current split mode of this node.
     * @param level the current level of this node.
     */
    public BSPTree(Vec2D[] bounds, int splitMode, int level) {
        super(12, 4);
        this.bounds = bounds;
        this.splitMode = splitMode;
        this.depth = level;

        items = new ArrayList<Shape>();
        children = new BSPTree[2];
    }

    public static int calculateDepthCap(ArrayList<Shape> shapes) {
        Vec2D[] aabb;
        double avgBound = 0;
        double wbs = 0; // world bounds
        for (Shape s : shapes) {
            if (!(s instanceof WorldBound)) {
                aabb = s.getAABBbounds();
                double t = 0;

                t += Math.abs(aabb[0].getX() - aabb[aabb.length - 1].getX()); // abs
                                                                              // of
                                                                              // min
                                                                              // -
                                                                              // max
                t += Math.abs(aabb[0].getY() - aabb[aabb.length - 1].getY());
                t /= 2;
                avgBound += t;
            }
            else
                wbs++;
        }
        avgBound /= (shapes.size() - wbs);

        int cap = 0;
        double base = Math.max(Phys2DMain.XRES, Phys2DMain.YRES);
        while (base > avgBound) {
            cap++;
            base /= 2.0;
        }
        cap = (int) Math.ceil(cap * 2.0);
        cap--;
        return cap;
    }

    // http://gamedevelopment.tutsplus.com/tutorials/quick-tip-use-quadtrees-to-detect-likely-collisions-in-2d-space--gamedev-374
    /**
     * Splits the current node into two separate children depending on the split
     * mode.
     */
    protected void split() {
        BSPTree c0, c1; // child 1, child 2

        Vec2D[] newBounds = new Vec2D[2]; //Temp var to store the new min-max bounds of each child.

        if (splitMode == VERTICAL_SPLIT) {

            double midX = (bounds[0].getX() + bounds[1].getX()) / 2.0;

            newBounds[0] = bounds[0].getCopy();
            newBounds[1] = new Vec2D(midX, bounds[1].getY());
            c0 = new BSPTree(newBounds, splitMode * -1, depth + 1); // left rect

            newBounds[0] = new Vec2D(midX, bounds[0].getY());
            newBounds[1] = bounds[1].getCopy();
            c1 = new BSPTree(newBounds, splitMode * -1, depth + 1); // right rect
        }

        else { //HORIZONTAL_SPLIT

            double midY = (bounds[0].getY() + bounds[1].getY()) / 2.0;

            newBounds[0] = bounds[0].getCopy();
            newBounds[1] = new Vec2D(bounds[1].getX(), midY);
            c0 = new BSPTree(newBounds, splitMode * -1, depth + 1); // bottom (top in GUI)

            newBounds[0] = new Vec2D(bounds[0].getX(), midY);
            newBounds[1] = bounds[1].getCopy();
            c1 = new BSPTree(newBounds, splitMode * -1, depth + 1); // top (bottom in gui)
        }

        children[0] = c0;
        children[1] = c1;

    }

    /**
     * Given the aabb of a shape, find out which child the shape get inserted
     * into.
     * 
     * @param aabb the min-max axis aligned bounding box of the shape being
     *            inserted.
     * @return the child the shape is inserted into. <br>
     *         <b><u>VERTICAL SPLIT</u></b> <br>
     *         <ul>
     *         <li>0 : Left side</li>
     *         <li>1 : Right side</li>
     *         </ul>
     *         <b><u>HORIZONTAL SPLIT</u></b>
     *         <ul>
     *         <li>0 : Top side</li>
     *         <li>1 : Bottom side</li>
     *         </ul>
     *         <li>-1 : Shape is on the dividing line.</li>
     */
    protected int getInsertionSideAABB(Vec2D[] aabb) {
        if (splitMode == VERTICAL_SPLIT) {
            double midX = (bounds[0].getX() + bounds[1].getX()) / 2.0;

            if (aabb[1].getX() < midX) // max of the aabb < center split line
                return 0; // LEFT SIDE
            else if (aabb[0].getX() >= midX) // min of aabb > center split line
                return 1; // RIGHT SIDE
        }
        else { // if horizontal split
            double midY = (bounds[0].getY() + bounds[1].getY()) / 2.0;

            if (aabb[1].getY() < midY)
                return 0; // TOP SIDE
            else if (aabb[0].getY() >= midY)
                return 1; // BOTTOM SIDE
        }

        return -1; // Shape is on dividing line
    }

    /**
     * Get which side the Shape s can be inserted on. 0 = left, 1 = right, -1 =
     * both
     * 
     * @param s
     * @return the side which the shape can be inserted on
     */
    protected int getInsertionSide(Shape s) {
        Vec2D[] aabb = s.getAABBbounds();
        return getInsertionSideAABB(aabb);
    }

    /**
     * Insert the shape into the current node
     * 
     * @param s the shape to insert
     */
    @Override
    public void insert(Shape s) {
        // if no children, try adding to parent

        if (children[0] != null) { // if there are children
            int insertionSide = getInsertionSide(s); // if there are children, check whether they will go left/top or right/bottom

            if (insertionSide != -1) { // if a suitable spot for the shape is found, insert it in there
                children[insertionSide].insert(s);
                return;
            }
            else { // The shape fits on both side, so insert it in there
                children[0].insert(s);
                children[1].insert(s);
                return;
            }
        }

        items.add(s); // If no children were found, add the s into the current node

        // But, now if we have overloaded this node, we need to split it down some more.
        // Also, we can only split if we havent exceeded the level_cap (depth cap).
        if (items.size() > MAX_ITEMS && depth < DEPTH_CAP) {
            split();

            // Now that there are children, we will offload the items into the children to get below the max_items threshold

            for (int i = items.size() - 1; i >= 0; i--) { // items size is not guaranteed

                // see which items can be offloaded into the children nodes
                int insertionSide = getInsertionSide(items.get(i));

                if (insertionSide != -1) { // if a valid insertion side is found, insert the item into that side
                    children[insertionSide].insert(items.remove(i));
                }
                else {
                    children[0].insert(items.get(i));
                    children[1].insert(items.remove(i));
                }
            }
        }
    }

    @Deprecated
    public Shape[] getPossibleColliders(Shape s) {
        ArrayList<Shape> possibleColliders = new ArrayList<Shape>(5);
        return getPossibleCollidersHelper(s, possibleColliders).toArray(new Shape[] {});
    }

    @Deprecated
    protected ArrayList<Shape> getPossibleCollidersHelper(Shape s, ArrayList<Shape> colliders) {
        int shapeSide = getInsertionSide(s); // Get which side the shape fits
                                             // into

        // if the shape can fit on a specific side, and if there are children,
        // go deeper into tree to find matches
        if (children[0] != null) {
            if (shapeSide != -1)
                children[shapeSide].getPossibleCollidersHelper(s, colliders);

            else { // if it doesnt fit on either side, check both sides
                children[0].getPossibleCollidersHelper(s, colliders);
                children[1].getPossibleCollidersHelper(s, colliders);
            }
        }
        // Now add everything at this current level. The above statement would
        // ensure the depths of the tree have been traversed
        colliders.addAll(items);

        return colliders;
    }

    @Deprecated
    public Shape[][] getCollisionGroups() {
        ArrayList<Shape[]> groups = new ArrayList<Shape[]>();
        getCollisionGroupsHelper(groups);
        return groups.toArray(new Shape[][] {});
    }

    protected void getCollisionGroupsHelper(ArrayList<Shape[]> groups) {
        if (children[0] == null) {
            if (items.size() > 1)
                groups.add(items.toArray(new Shape[] {}));
        }
        else {
            children[0].getCollisionGroupsHelper(groups);
            children[1].getCollisionGroupsHelper(groups);
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
        ArrayList<Shape[]> collisionGroups = new ArrayList<Shape[]>();
        getCollisionGroupsHelper(collisionGroups);
        return collisionGroups;
    }

    /**
     * Cleans this tree by reseting it's children and items.
     */
    @Override
    public void refresh() {
        items = new ArrayList<Shape>();
        children = new BSPTree[2];
    }

    @Override
    public void draw(Graphics2D g2d) {
        Color t = g2d.getColor();

        g2d.setColor(Color.ORANGE);

        Vec2D com = new Vec2D((bounds[0].getX() + bounds[1].getX()) / 2.0, (bounds[0].getY() + bounds[1].getY()) / 2.0);

        g2d.drawString(depth + " " + items.size(), (int) com.getX() - 10, Phys2DMain.YRES - (int) (com.getY() + 5));
        if (children[0] != null) {
            if (splitMode == VERTICAL_SPLIT) {
                g2d.drawLine((int) com.getX(), Phys2DMain.YRES - (int) bounds[0].getY(), (int) com.getX(),
                        Phys2DMain.YRES - (int) bounds[1].getY());
            }
            else { //Horizontal split
                g2d.drawLine((int) bounds[0].getX(), Phys2DMain.YRES - (int) com.getY(), (int) bounds[1].getX(),
                        Phys2DMain.YRES - (int) com.getY());
            }
            for (BSPTree child : children) {
                child.draw(g2d);
            }
        }

        g2d.setColor(t);

    }
}