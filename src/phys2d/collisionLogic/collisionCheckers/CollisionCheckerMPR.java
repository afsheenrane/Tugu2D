package phys2d.collisionLogic.collisionCheckers;

import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;

/**
 * This class contains methods which detect collision and return minimum
 * displacement to unstick shapes, using the Minkowski Portal Refinement
 * algorithm.
 * 
 * @author Afsheen
 *
 */
public final class CollisionCheckerMPR extends CollisionChecker {

    private final CollisionCheckerGJKEPA2 gjkTool = new CollisionCheckerGJKEPA2();

    /**
     * Uses the MPR algorithm to detect whether a collision has occurred between
     * shapes s1 and s2.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @return true if the shapes are colliding, false otherwise.
     */
    @Override
    public boolean isColliding(Shape s1, Shape s2) {
        SimplexCollisionInfo mprInfo = new SimplexCollisionInfo();
        computeSimplex(s1, s2, mprInfo);

        return mprInfo.isColliding;
    }

    /**
     * If the shapes are colliding, return the minimum displacement to unstick
     * them, otherwise, return the minimum displacement between the two shapes.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     */
    @Override
    public SimplexCollisionInfo getCollisionResolution(Shape s1, Shape s2) {
        SimplexCollisionInfo mprInfo = new SimplexCollisionInfo();
        computeSimplex(s1, s2, mprInfo);

        if (mprInfo.isColliding)
            computeCollisionResolution(s1, s2, mprInfo);

        else
            computeMinimumDisplacement(s1, s2, mprInfo);

        return mprInfo;
    }

    private void computeCollisionResolution(Shape s1, Shape s2, SimplexCollisionInfo mprInfo) {
        mprInfo.simplex.remove(0);
        mprInfo.simplex.add(support(s1, s2, mprInfo.dir.getNegated()));
        gjkTool.computeCollisionResolutionEPA(s1, s2, mprInfo);
    }

    /**
     * Evolve the simplex using MPR into it's final state.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param mprInfo the structure where information about the MPR run is
     *            stored.
     */
    private void computeSimplex(Shape s1, Shape s2, SimplexCollisionInfo mprInfo) {

        //System.out.println("diff: " + LinePolyTools.polyDifference(s1, s2));

        int c = 0;

        Vec2D RA, portal, RAnorm, RO;

        mprInfo.simplex.add(Vec2D.sub(s1.getCOM(), s2.getCOM())); // R: V0

        RO = mprInfo.simplex.get(0).getNegated();

        mprInfo.dir = RO;
        mprInfo.simplex.add(support(s1, s2, mprInfo.dir)); // A: V1

        RA = Vec2D.sub(mprInfo.simplex.get(1), mprInfo.simplex.get(0)); // A - R
        RAnorm = RA.getNormal();

        // Find which side origin is on.
        if (RAnorm.dotProduct(mprInfo.dir) <= 0) {
            RAnorm.negate();
        }
        mprInfo.dir = RAnorm;

        mprInfo.simplex.add(support(s1, s2, mprInfo.dir)); // B: V2

        // REFINEMENT PHASE.
        while (c++ <= 50) {

            // AB
            portal = Vec2D.sub(mprInfo.simplex.get(2), mprInfo.simplex.get(1)); //B - A

            // Check if the origin is inside the simplex.
            /*
             * We already know that the origin is within the swept angle of RA
             * and RB. So we really only need to check if it's on the correct
             * side of the portal. Papers and the source algorithm say we need
             * to perform a full on triangle check, but I really see no reason
             * doing 2 redundant checks because we already knows it's within 2
             * of the 3 triangle edges.
             */
            // Check if origin is on correct side of portal.
            Vec2D newPt, AO = mprInfo.simplex.get(1).getNegated();
            Vec2D portalNorm = portal.getNormal(); // Normal axis of the portal. Still need to guarantee that it points outwards.

            if (portalNorm.dotProduct(RA.getNegated()) > 0) //This guarantees that portalNorm points outwards.
                portalNorm.negate();

            // If the origin is outside the portal
            if (AO.dotProduct(portalNorm) >= 0) {
                mprInfo.dir = portalNorm;
                newPt = support(s1, s2, mprInfo.dir);

                // See if the new point is past the origin.
                if (newPt.dotProduct(mprInfo.dir) <= 0) { // If not past origin
                    mprInfo.isColliding = false;
                    return;
                }
                else {

                    // REFINE THE PORTAL.
                    // Check to see which point to discard from the simplex.
                    Vec2D RC = Vec2D.sub(newPt, mprInfo.simplex.get(0));

                    // If RO and RA are on the same side (the below checks their sign. Same signs multiplied will be +ve.
                    if (RC.perpDotProduct(RO) * RC.perpDotProduct(RA) > 0) {
                        //Discard B.
                        mprInfo.simplex.set(2, newPt);
                    }
                    else { //Discard A
                        mprInfo.simplex.set(1, newPt);
                    }
                }
            }
            else {
                // Else, it is inside the portal.
                mprInfo.isColliding = true;
                return;
            }

        }
        System.err.println("MPR COMPUTE SIMPLEX FAILURE.");
        System.exit(1);
    }

    /**
     * Find the minimum displacement between the two shapes if they are not
     * colliding. <br>
     * This method reuses GJKv2's method of finding closest displacement. The
     * only difference is the starting simplex was generated by different
     * algorithms.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param mprInfo the state of the simplex and the last search direction
     *            after the MPR algorithm searched for the origin.
     */
    protected void computeMinimumDisplacement(Shape s1, Shape s2, SimplexCollisionInfo mprInfo) {

        final double TOL = 0.1;

        mprInfo.simplex.remove(0); // Remove the COM point.

        // start the march towards the origin till the tol is reached.
        while (true) {

            // Find closest point on line segment to the origin. 
            Vec2D AB = Vec2D.sub(mprInfo.simplex.get(0), mprInfo.simplex.get(1)); //B - A
            Vec2D AO = mprInfo.simplex.get(1).getNegated();

            //Find the closest point on the line segment A-B
            Vec2D closestPt = AO.vecProjection(AB);

            closestPt.add(mprInfo.simplex.get(1));

            // Now check if closestPt was outside of line seg.
            Vec2D ACl = Vec2D.sub(closestPt, mprInfo.simplex.get(1));
            double ABdotACl = AB.dotProduct(ACl);

            if (ABdotACl <= 0) {
                closestPt = mprInfo.simplex.get(1).getCopy();
            }
            else if (ABdotACl >= AB.getSquaredLength()) {
                closestPt = mprInfo.simplex.get(0).getCopy();
            }
            // Closest point is now set.

            if (closestPt.equals(Vec2D.ORIGIN)) {
                mprInfo.dir = Vec2D.ORIGIN;
                return;
            }

            // Find the direction of the origin from the closest point.
            closestPt.negate();
            mprInfo.dir = closestPt;

            Vec2D newPt;
            newPt = support(s1, s2, mprInfo.dir);

            // Check if the new support is actually making progress towards the origin.
            if (Vec2D.sub(newPt, mprInfo.simplex.get(1)).dotProduct(mprInfo.dir) <= TOL ||
                    Vec2D.sub(newPt, mprInfo.simplex.get(0)).dotProduct(mprInfo.dir) <= TOL) { // If no progress
                mprInfo.dir = closestPt;
                return;
            }

            // If progress was made, replace a bad point in the simplex with the
            // new support.
            if (mprInfo.simplex.get(0).getSquaredLength() > mprInfo.simplex.get(1).getSquaredLength())
                mprInfo.simplex.set(0, newPt);
            else
                mprInfo.simplex.set(1, newPt);
        }
    }

    /**
     * Returns the support point of the minkowski difference of s1 and s2 in
     * direction dir.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param dir the direction to get the support point in.
     * @return the corresponding support mapping of dir for s1 - s2.
     */
    private Vec2D support(Shape s1, Shape s2, Vec2D dir) {
        return (Vec2D.sub(s1.getMax(dir), s2.getMin(dir)));
    }

}
