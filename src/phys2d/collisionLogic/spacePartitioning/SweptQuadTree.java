package phys2d.collisionLogic.spacePartitioning;

import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;

public class SweptQuadTree extends QuadTree {

    private final double dt;

    /**
     * Create a new SweptQuadTree with the given bounds, the current level of
     * the node, and the current physics delta time (to compute AABB's of the
     * shapes).
     * 
     * @param bounds the bounding rectangle that this node covers in the game
     *            world, <b>in MIN-MAX notation<b>.
     * @param level the current level of this node.
     * @param dt the physics delta time of the simulation.
     */
    public SweptQuadTree(Vec2D[] bounds, int level, double dt) {
        super(bounds, level);
        this.dt = dt;
    }

    /**
     * Splits the current node into four separate children.
     */
    @Override
    protected void split() {

        Vec2D center = new Vec2D((bounds[0].getX() + bounds[1].getX()) / 2.0,
                (bounds[0].getY() + bounds[1].getY()) / 2.0);

        children[0] = new SweptQuadTree(new Vec2D[] { bounds[0].getCopy(), center.getCopy() }, depth + 1, dt); //BL

        children[1] = new SweptQuadTree(new Vec2D[] { //TL
                new Vec2D(bounds[0].getX(), center.getY()),
                new Vec2D(center.getX(), bounds[1].getY())
        }, depth + 1, dt);

        children[2] = new SweptQuadTree(new Vec2D[] { center.getCopy(), bounds[1].getCopy() }, depth + 1, dt); //TR

        children[3] = new SweptQuadTree(new Vec2D[] { //BR
                new Vec2D(center.getX(), bounds[0].getY()),
                new Vec2D(bounds[1].getX(), center.getY())
        }, depth + 1, dt);
    }

    /**
     * Inserts the shape s into all possible children, by taking into account
     * it's full movement over the course of the current tick.
     */
    @Override
    protected void insertShapeIntoChildren(Shape s) {
        Vec2D[] aabb = s.getSweptAABBbounds(dt);
        for (int i = 0; i < children.length; i++) {
            if (childCanContainAABB(children[i], aabb)) {
                children[i].insert(s);
            }
        }
    }

}
