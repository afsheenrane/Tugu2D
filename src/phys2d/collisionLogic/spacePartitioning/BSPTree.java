package phys2d.collisionLogic.spacePartitioning;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.List;

import phys2d.Phys2DMain;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Rectangle;
import phys2d.entities.shapes.polygons.WorldBound;

public class BSPTree {

    public static final int HORIZONTAL_SPLIT = -1;
    public static final int VERTICAL_SPLIT = 1;

    protected static final int MAX_ITEMS = 4;
    protected static int DEPTH_CAP = 12; // TODO make protected from public

    protected int depth = 0;

    protected List<Shape> items;
    protected final Rectangle bounds;
    protected BSPTree[] children;
    protected final int splitMode;

    public BSPTree(Rectangle bounds, int splitMode, int level) {
        this.bounds = bounds;
        this.splitMode = splitMode;
        this.depth = level;

        items = new ArrayList<Shape>();
        children = new BSPTree[2];
    }

    public static void calculateDepthCap(ArrayList<Shape> shapes) {
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
        DEPTH_CAP = cap;
    }

    /**
     * Clears this BSPTree and all its children
     */
    public void clear() {
        items.clear();

        for (BSPTree child : children) {
            if (child != null) {
                child.clear();
                child = null;
            }
        }
    }

    // http://gamedevelopment.tutsplus.com/tutorials/quick-tip-use-quadtrees-to-detect-likely-collisions-in-2d-space--gamedev-374

    public void split() {
        BSPTree c0, c1; // child 1, child 2

        Rectangle newBounds;
        Vec2D com = bounds.getCOM().getCopy();

        if (splitMode == VERTICAL_SPLIT) {

            com.setX(com.getX() - (bounds.getLength() / 4));
            newBounds = new Rectangle(com.getCopy(), bounds.getLength() / 2,
                    bounds.getHeight()); // doing com copy because com is being aliased. not copying newbounds because it is being being re-assigned
            c0 = new BSPTree(newBounds, splitMode * -1, depth + 1); // left rect

            com.setX(com.getX() + bounds.getLength() / 2);
            newBounds = new Rectangle(com.getCopy(), bounds.getLength() / 2,
                    bounds.getHeight());
            c1 = new BSPTree(newBounds, splitMode * -1, depth + 1); // right rect
        }
        else {

            com.setY(com.getY() - (bounds.getHeight() / 4));
            newBounds = new Rectangle(com.getCopy(), bounds.getLength(),
                    bounds.getHeight() / 2);
            c0 = new BSPTree(newBounds, splitMode * -1, depth + 1); // bottom (top in GUI)

            com.setY(com.getY() + bounds.getHeight() / 2);
            newBounds = new Rectangle(com.getCopy(), bounds.getLength(),
                    bounds.getHeight() / 2);
            c1 = new BSPTree(newBounds, splitMode * -1, depth + 1); // top (bottom in gui)
        }

        children[0] = c0;
        children[1] = c1;

    }

    protected int getInsertionSideAABB(Vec2D[] aabb) {
        if (splitMode == VERTICAL_SPLIT) {
            if (aabb[1].getX() < bounds.getCOM().getX()) // max of the aabb < center split line
                return 0; // LEFT SIDE
            else if (aabb[0].getX() >= bounds.getCOM().getX()) // min of aabb > center split line
                return 1; // RIGHT SIDE
        }
        else { // if horizontal split
            if (aabb[1].getY() < bounds.getCOM().getY())
                return 0; // TOP SIDE
            else if (aabb[0].getY() >= bounds.getCOM().getY())
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

            // if(children[0] == null){ //if there are no children, split the current tree.
            split();

            // Now that there are children, we will offload the items into the children to get below the max_items threshold

            for (int i = items.size() - 1; i >= 0; i--) { // items size is not guaranteed

                // see which items can be offloaded into the children nodes
                int insertionSide = getInsertionSide(items.get(i));
                int x = 0;
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

    public Shape[] getPossibleColliders(Shape s) {
        ArrayList<Shape> possibleColliders = new ArrayList<Shape>(5);
        return getPossibleCollidersHelper(s, possibleColliders).toArray(
                new Shape[] {});
    }

    protected ArrayList<Shape> getPossibleCollidersHelper(Shape s,
            ArrayList<Shape> colliders) {
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
    public ArrayList<Shape[]> getPossibleCollisions() {
        ArrayList<Shape[]> collisionGroups = new ArrayList<Shape[]>();
        getCollisionGroupsHelper(collisionGroups);
        return collisionGroups;
    }

    /**
     * Cleans this tree by reseting it's children and items.
     */
    public void refresh() {
        items = new ArrayList<Shape>();
        children = new BSPTree[2];
    }

    public void draw(Graphics2D g2d) {
        Color t = g2d.getColor();

        g2d.setColor(Color.ORANGE);

        Vec2D com = bounds.getCOM();
        g2d.drawString(depth + " " + items.size(), (int) com.getX() - 10, Phys2DMain.YRES - (int) (com.getY() + 5));
        if (children[0] != null) {
            if (splitMode == VERTICAL_SPLIT)
                g2d.drawLine((int) com.getX(),
                        Phys2DMain.YRES - (int) (com.getY() - (bounds.getHeight() / 2)),
                        (int) com.getX(),
                        Phys2DMain.YRES - (int) (com.getY() + (bounds.getHeight() / 2)));

            else
                g2d.drawLine((int) (com.getX() - (bounds.getLength() / 2)),
                        Phys2DMain.YRES - (int) com.getY(),
                        (int) (com.getX() + (bounds.getLength() / 2)),
                        Phys2DMain.YRES - (int) com.getY());

            for (BSPTree child : children) {
                child.draw(g2d);
            }
        }

        g2d.setColor(t);

    }
}