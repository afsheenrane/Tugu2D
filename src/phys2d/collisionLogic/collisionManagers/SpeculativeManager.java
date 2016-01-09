package phys2d.collisionLogic.collisionManagers;

import java.util.ArrayList;

import phys2d.Phys2DMain;
import phys2d.collisionLogic.collisionCheckers.CollisionCheckerGJKEPA;
import phys2d.collisionLogic.spacePartitioning.BSPTree;
import phys2d.collisionLogic.spacePartitioning.SweptBSPTree;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.WorldBound;

public final class SpeculativeManager extends CollisionManager {

    private BSPTree collisionTree;

    public SpeculativeManager(double dt) {
        super(dt);
    }

    @Override
    public BSPTree getCollisionTree() {
        return collisionTree;
    }

    private void checkAndResolveCollisions(Shape[] shapes) {

        collisionTree = new SweptBSPTree(new Vec2D[] { new Vec2D(-25, -25),
                new Vec2D(Phys2DMain.XRES + 25, Phys2DMain.YRES + 25) }, BSPTree.HORIZONTAL_SPLIT, 1, dt);

        ArrayList<Shape[]> collidedPairs = new ArrayList<Shape[]>();

        for (Shape s : shapes) {
            collisionTree.insert(s);
        }

        Shape[][] collisionGroups = collisionTree.getCollisionGroups();

        for (Shape[] group : collisionGroups) {

            for (int i = 0; i < group.length; i++) { // perform a "brute force"
                                                     // collision check
                for (int j = i + 1; j < group.length; j++) {
                    if (!(group[i] instanceof WorldBound && group[j] instanceof WorldBound) && // world bounds dont collide with each other
                            !deepContains(collidedPairs, new Shape[] { group[i], group[j] })) {
                        resolveCollision(group[i], group[j]);
                        collidedPairs.add(new Shape[] { group[i], group[j] });
                        collidedPairs.add(new Shape[] { group[j], group[i] });
                    }
                }
            }
        }
    }

    private void computeForce(Shape s1, Shape s2, double relNormSpeed, Vec2D dispUnit) {
        System.out
                .println("doing a full resolution*************************************************************************");
        System.out.println(s1 + "\n" + s2);
        System.out.println("dispUnit: " + dispUnit);

        // the speed that needs to be removed from the to ensure no discrete
        // collision
        // double overSpeed = Math.abs(relNormSpeed) - disp.getLength();

        // react to the collision with forces.
        double restitution = Math.min(s1.getMaterial().getRestitution(), s2.getMaterial().getRestitution());

        // relVel.scaleBy(1.0 / dt);
        Vec2D force = Vec2D.getScaled(dispUnit, -(1 + restitution) * relNormSpeed * (1.0 / dt));
        force.scaleBy(1.0 / (s1.getInvMass() + s2.getInvMass()));

        s1.addForce(force);
        s2.addForce(Vec2D.getNegated(force));
    }

    /**
     * Shift the shapes so that they are in contact with each other.
     * 
     * @param s1 shape 1
     * @param s2 shape 2
     * @param disp displacement between the shapes
     * @param dispUnit unit displacement vector
     */
    private void computeShift(Shape s1, Shape s2, Vec2D disp, Vec2D dispUnit) {
        // remove the speeds from the two object. (Use a weighted average of the
        // masses or speed or something))
        double s1ratio, s2ratio;
        double s1RelSpeed = s1.getVelocity().dotProduct(dispUnit);
        double s2RelSpeed = s2.getVelocity().dotProduct(dispUnit.getNegated());
        double netRelSpeed = s1RelSpeed + s2RelSpeed;

        s1ratio = s1RelSpeed / netRelSpeed;
        s2ratio = 1.0 - s1ratio; // Doing this to reduce numerical errors from
                                 // division.

        // fix the translation
        // s1.translate(disp.getNegated());

        s1.translate(disp.getScaled(s1ratio));
        s2.translate(disp.getScaled(-s2ratio));
    }

    private void resolveCollision(Shape s1, Shape s2) {

        // TODO clean this shit up.

        if (CollisionCheckerGJKEPA.isColliding(s1, s2)) {
            // System.out.println("collision!!!");
            // System.exit(0);
        }

        for (int i = 0; i < 2; i++) {
            Object[] collisionInfo = CollisionCheckerGJKEPA.hybridGJKSolver(s1, s2);
            Vec2D disp = ((Vec2D) collisionInfo[0]).getNegated();
            boolean isColliding = (boolean) collisionInfo[1];
            // TODO incorp isColliding

            if (isColliding) {
                unstickShapes(s1, s2, disp);
                disp.negate(); // Doing this because the disp would originally
                               // have been reversed since the shapes were
                               // colliding
            }

            // System.out.println("\ndisp: " + disp);
            // System.out.println("dispLen: " + disp.getLength());

            // Move the shapes back in time by 1 timestep.
            if (disp.equals(Vec2D.ORIGIN)) {
                // System.out.println("origined");
                s1.translate(s1.getVelocity().getScaled(-dt));
                s2.translate(s2.getVelocity().getScaled(-dt));
                continue;
            }

            Vec2D dispUnit = disp.getNormalized();
            Vec2D relVel = Vec2D.sub(s1.getVelocity(), s2.getVelocity()); // making
                                                                          // s2
                                                                          // stationary
            relVel.scaleBy(dt);
            // System.out.println("relVel: " + relVel);

            double relNormSpeed = relVel.dotProduct(dispUnit); // relative speed
                                                               // along
                                                               // collision
                                                               // normal
            // System.out.println("relNormSpeed: " + relNormSpeed);
            // System.out.println("i: " + i);
            // if a collision is imminent next frame. That is, if the relative
            // normal velocity (px/dt) is greater than the distance between the
            // objects
            // Unless ofcourse a collision has already happened, then immediate
            // resolve the collision.
            if (i == 0) {
                if (relNormSpeed >= disp.getLength() || isColliding) {
                    System.out.println("disp: " + disp);
                    computeForce(s1, s2, relNormSpeed, dispUnit);

                    // Dont compute shift if there were colliding.
                    // Because the unsticker will already keep them just
                    // touching. Dont overcompensate.
                    if (!isColliding)
                        computeShift(s1, s2, disp, dispUnit);
                }
                return;
            }

            else if (i == 1) {
                if (relNormSpeed > 0) {
                    // Translate the shapes back
                    s1.translate(s1.getVelocity().getScaled(dt));
                    s2.translate(s2.getVelocity().getScaled(dt));
                    computeForce(s1, s2, relNormSpeed, dispUnit);
                }
                return;
            }

        }

        /*
         * //System.out.println("\nColliders:\n" + s1 + " +\n" + s2 + "\n");
         * for(int i = 0; i < 2; i++){ Vec2D disp =
         * CollisionCheckerGJKEPA.getDisplacementBetweenShapes(s1,
         * s2).getNegated(); // System.out.println("\ndisp: " + disp); //
         * System.out.println("dispLen: " + disp.getLength());
         * 
         * //Move the shapes back in time by 1 timestep.
         * if(disp.equals(Vec2D.ORIGIN)){ // System.out.println("origined");
         * s1.translate(s1.getVelocity().getScaled(-dt));
         * s2.translate(s2.getVelocity().getScaled(-dt)); continue; }
         * 
         * Vec2D dispUnit = disp.getNormalized(); Vec2D relVel =
         * Vec2D.sub(s1.getVelocity(), s2.getVelocity()); //making s2 stationary
         * relVel.scaleBy(dt); //System.out.println("relVel: " + relVel);
         * 
         * double relNormSpeed = relVel.dotProduct(dispUnit); //relative speed
         * along collision normal // System.out.println("relNormSpeed: " +
         * relNormSpeed); // System.out.println("i: " + i); //if a collision is
         * imminent next frame. That is, if the relative normal velocity (px/dt)
         * is greater than the distance between the objects if(i == 0){
         * if(relNormSpeed >= disp.getLength()) { System.out.println("disp: " +
         * disp); computeForce(s1, s2, relNormSpeed, dispUnit); computeShift(s1,
         * s2, disp, dispUnit); } return; }
         * 
         * else if(i == 1){ if(relNormSpeed > 0) { //Translate the shapes back
         * s1.translate(s1.getVelocity().getScaled(dt));
         * s2.translate(s2.getVelocity().getScaled(dt)); computeForce(s1, s2,
         * relNormSpeed, dispUnit); } return; }
         * 
         * }
         */
        // System.out.println("End for rectified");
        s1.translate(s1.getVelocity().getScaled(dt + dt)); // 2.0 * dt
        s2.translate(s2.getVelocity().getScaled(dt + dt));
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

        System.out.println("\nunsticker" + "\n" + s1 + "\n" + s2 + "\n" + resolution);

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

    protected void manageCollisions(Shape[] entities) {
        checkAndResolveCollisions(entities);

    }

    @Override
    public void runManager(ArrayList<Shape> entities) {

        // addWorldForces(entities);

        manageCollisions(entities.toArray(new Shape[] {}));

        moveEntities(entities);
    }

    @Override
    protected void manageCollisions(ArrayList<Shape> entities) {
        // TODO Auto-generated method stub

    }

}
