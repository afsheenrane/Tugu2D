package phys2d.collisionLogic.collisionManagers;

import java.util.ArrayList;

import phys2d.Phys2DMain;
import phys2d.collisionLogic.collisionCheckers.CollisionCheckerGJKEPA;
import phys2d.collisionLogic.spacePartitioning.BSPTree;
import phys2d.collisionLogic.spacePartitioning.SweptBSPTree;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.WorldBound;

public final class DiscreteManager extends CollisionManager {

    private BSPTree collisionTree;

    public DiscreteManager(double dt) {
        super(dt);
    }

    public BSPTree getCollisionTree() {
        return collisionTree;
    }

    private void checkAndResolveCollisions(Shape[] shapes) {

        // Using BSPTree
        collisionTree = new SweptBSPTree(new Vec2D[] { new Vec2D(-25, -25),
                new Vec2D(Phys2DMain.XRES + 25, Phys2DMain.YRES + 25) }, BSPTree.HORIZONTAL_SPLIT, 1, dt);

        ArrayList<Shape[]> collidedPairs = new ArrayList<Shape[]>();

        for (Shape s : shapes) { // populate the space partitioning tree with entities
            collisionTree.insert(s);
        }

        int checks = 0;

        // Get the collection of all shapes which are close enough to each other
        // to warrant further testing
        Shape[][] collisionGroups = collisionTree.getCollisionGroups();
        for (Shape[] group : collisionGroups) {

            for (int i = 0; i < group.length; i++) { // perform a "brute force"
                                                     // collision check
                for (int j = i + 1; j < group.length; j++) {

                    if (!(group[i] instanceof WorldBound && group[j] instanceof WorldBound) // if two pairs aren't worldbounds
                            && !deepContains(collidedPairs, new Shape[] { group[i], group[j] })) { // check if they have been collided before

                        // check for collisions and apply impulse if needed
                        resolveCollision(group[i], group[j]);
                        collidedPairs.add(new Shape[] { group[i], group[j] });
                        collidedPairs.add(new Shape[] { group[j], group[i] });
                        checks++;
                    }
                }
            }
        }

        // System.out.println(checks);
    }

    /**
     * Translates colliding shapes out of each other
     * 
     * @param s1 the first shape
     * @param s2 the second shape
     * @param resolution the penetration vector between the shapes
     */
    private void unstickShapes(Shape s1, Shape s2, Vec2D resolution) {
        double totalMass = s1.getMass() + s2.getMass();

        double s1ratio, s2ratio;
        Vec2D midPt, s1RelPos, s2RelPos;

        midPt = Vec2D.add(s1.getCOM(), s2.getCOM());
        midPt.scaleBy(0.5); // (s1 + s2) / 2

        s1RelPos = Vec2D.sub(s1.getCOM(), midPt);
        s2RelPos = Vec2D.sub(s2.getCOM(), midPt);

        if (s1 instanceof WorldBound) {
            s1ratio = 0;
            s2ratio = 1;
        }
        else if (s2 instanceof WorldBound) {
            s1ratio = 1;
            s2ratio = 0;
        }
        else {
            s1ratio = s1.getMass() / totalMass;
            s2ratio = s2.getMass() / totalMass;
        }

        // System.out.println("unsticker");

        // first deal with s1
        Vec2D trans = resolution.getCopy();
        trans.scaleBy(s1ratio); // calculate the magnitude of the unstick vector
        // this second scale decides the direction of the translation (positive
        // or negative)
        trans.scaleBy(Math.signum(s1RelPos.dotProduct(resolution)));
        // System.out.println("s1 trans: " + trans);
        s1.translate(trans);

        trans = resolution.getCopy();
        trans.scaleBy(s2ratio);
        trans.scaleBy(Math.signum(s2RelPos.dotProduct(resolution)));
        // System.out.println("s2 trans: " + trans);
        s2.translate(trans);

    }

    private void resolveCollision(Shape s1, Shape s2) {
        // TODO clean this up.

        // Find the axis of collision between the two shapes
        Vec2D collisionAxis = CollisionCheckerGJKEPA.getCollisionResolutionGJKEPA(s1, s2);

        if (!collisionAxis.equals(Vec2D.ORIGIN)) { // if they are colliding

            unstickShapes(s1, s2, collisionAxis);

            collisionAxis.normalize();

            Vec2D relVel = Vec2D.sub(s1.getVelocity(), s2.getVelocity());

            // speed along the collision normal
            double normalSpeed = Vec2D.dotProduct(relVel, collisionAxis);

            if (normalSpeed < 0) {
                System.out.println("early return from discrete (145)");
                return;
            }

            double restitution = Math.min(s1.getMaterial().getRestitution(), s2.getMaterial().getRestitution());

            Vec2D force = Vec2D.getScaled(collisionAxis, -(1 + restitution) * normalSpeed);

            force.scaleBy(1.0 / (s1.getInvMass() + s2.getInvMass()));

            s1.addForce(force);
            s2.addForce(Vec2D.getNegated(force));

        }
    }

    protected void manageCollisions(Shape[] entities) {
        checkAndResolveCollisions(entities);
    }

    @Override
    public void runManager(ArrayList<Shape> entities) {

        addWorldForces(entities);

        manageCollisions(entities.toArray(new Shape[] {}));

        moveEntities(entities);
    }

    @Override
    protected void manageCollisions(ArrayList<Shape> entities) {
        // TODO Auto-generated method stub

    }

}
