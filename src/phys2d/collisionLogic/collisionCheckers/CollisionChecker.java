package phys2d.collisionLogic.collisionCheckers;

import phys2d.entities.shapes.Shape;

public abstract class CollisionChecker {
    public abstract CollisionInfo getCollisionResolution(Shape s1, Shape s2);

    public abstract boolean isColliding(Shape s1, Shape s2);
}
