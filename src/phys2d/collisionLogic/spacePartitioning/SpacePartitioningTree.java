package phys2d.collisionLogic.spacePartitioning;

import java.awt.Graphics2D;
import java.util.ArrayList;

import phys2d.entities.shapes.Shape;

public abstract class SpacePartitioningTree {

    protected final int DEPTH_CAP;
    protected final int MAX_ITEMS;

    /**
     * Initializes the depth cap and maximum items per node, for this space
     * partitioning tree.
     * 
     * @param DEPTH_CAP the maximum depth of the tree.
     * @param MAX_ITEMS the maximum items per node of the tree.
     */
    protected SpacePartitioningTree(int DEPTH_CAP, int MAX_ITEMS) {
        this.DEPTH_CAP = DEPTH_CAP;
        this.MAX_ITEMS = MAX_ITEMS;
    }

    /**
     * Insert the shape into the current node of this space partitioning tree.
     * 
     * @param s the shape to insert.
     */
    public abstract void insert(Shape s);

    /**
     * Cleans this tree by reseting it's children and items.
     */
    public abstract void refresh();

    public abstract ArrayList<Shape[]> getPossibleCollisions();

    public abstract void draw(Graphics2D g2d);

}
