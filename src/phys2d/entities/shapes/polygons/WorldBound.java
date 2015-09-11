package phys2d.entities.shapes.polygons;

import phys2d.entities.Material;
import phys2d.entities.Vec2D;

public final class WorldBound extends Rectangle {

    /**
     * Initialize an immovable world boundary.
     * 
     * @param center the center of the shape.
     * @param length the x extent of the shape.
     * @param height the y extent of the shape.
     * @param angle the current rotation of the shape.
     */
    public WorldBound(Vec2D center, double length, double height,
            double angle) {
        super(center, length, height, angle, Material.REFLECTIUM);
        setMass(0);
    }

    /**
     * Initialize an immovable world boundary.
     * 
     * @param center the center of the shape.
     * @param length the x extent of the shape.
     * @param height the y extent of the shape.
     */
    public WorldBound(Vec2D center, double length, double height) {
        this(center, length, height, 0);
    }

    /**
     * Initialize an immovable AABB world boundary using mix max form.
     * 
     * @param min the minimum x and y position of the rectangle.
     * @param max the maximum x and y position of the rectangle.
     */
    public WorldBound(Vec2D min, Vec2D max) {
        this(new Vec2D((min.getX() + max.getX()) / 2,
                (min.getY() + max.getY()) / 2), max.getX() - min.getX(),
                max.getY() - min.getY());
    }

    @Override
    public Vec2D getVelocity() {
        return Vec2D.ORIGIN;
    }

    @Override
    public void translate(Vec2D trans) {
        // Do Nothing.
    }

    @Override
    public void move(double dt) {
        // Do nothing. Because world bounds are immovable.
    }

    @Override
    public void setVelocity(Vec2D velocity) {
        // Do nothing. World bounds are immovable.
    }
}
