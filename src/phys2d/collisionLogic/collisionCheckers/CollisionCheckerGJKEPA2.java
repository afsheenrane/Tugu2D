package phys2d.collisionLogic.collisionCheckers;

import phys2d.collisionLogic.tools.LinePolyTools;
import phys2d.collisionLogic.tools.MiscTools;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Polygon;

/**
 * A better written version of GJK and EPA with a focus on speed. Tests show it
 * to be around 3 times faster than v1.
 * 
 * @author afsheen
 *
 */
public final class CollisionCheckerGJKEPA2 {

    private final int CLOCKWISE_WINDING = 1;
    private final int ANTICLOCKWISE_WINDING = -1;

    /**
     * Using GJK, return the result of the algorithm on the two shapes.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @return a structure containing the final state of the simplex, the last
     *         search direction, and whether the shapes are colliding or not.
     */
    private SimplexDirStruct computeSimplex(Shape s1, Shape s2) {

        //System.out.println(LinePolyTools.polyDifference(s1, s2));

        Vec2D newPt;
        SimplexDirStruct gjkInfo = new SimplexDirStruct();

        int count = 0;

        gjkInfo.dir = Vec2D.sub(s1.getCOM(), s2.getCOM()); // COM of s1 - s2.
        gjkInfo.dir.negate(); // And point that towards the origin.

        gjkInfo.simplex.add(support(s1, s2, gjkInfo.dir));

        gjkInfo.dir = gjkInfo.simplex.get(0).getNegated();

        while (count < 50) {
            newPt = support(s1, s2, gjkInfo.dir);

            // If the new point is not past the origin, then the origin cannot be encapsulated.
            if (newPt.dotProduct(gjkInfo.dir) <= 0) {
                gjkInfo.isColliding = false;
                return gjkInfo;
            }

            gjkInfo.simplex.add(newPt);
            evolveSimplex(gjkInfo);
            if (gjkInfo.isColliding) {
                return gjkInfo;
            }

            count++;

        }

        System.err.println("GJK TOL failure.");
        System.exit(1);
        gjkInfo.isColliding = false;
        return gjkInfo;

    }

    /**
     * Modifies the simplex according to it's current characteristics and change
     * the search direction if needed.
     * 
     * @param gjkInfo the current state of the algorithm.
     */
    private void evolveSimplex(SimplexDirStruct gjkInfo) {

        switch (gjkInfo.simplex.size()) {
            case 2:
                computeLineSimplex(gjkInfo);
                break;
            case 3:
                computeTriangleSimplex(gjkInfo);
                break;
            default:
                System.err.println("Simplex size error: " + gjkInfo.simplex.size());

                System.exit(0);
                break;
        }
    }

    /**
     * Compute the new search direction and new simplex if it is currently a
     * line. <br>
     * The first point cannot be the closest feature to the origin. This is
     * because it was already deduced that the newly added point is in the
     * direction of the origin. Therefore, we only need to check if the new
     * point is closest to the origin, or whether the line body is closest.
     * 
     * @param gjkInfo the current state of the algorithm.
     */
    private void computeLineSimplex(SimplexDirStruct gjkInfo) {

        // Line: B-------------A
        // B=0,A=1

        Vec2D AB, AO;

        AB = Vec2D.sub(gjkInfo.simplex.get(0), gjkInfo.simplex.get(1)); // B - A
        AO = gjkInfo.simplex.get(1).getNegated();

        // If the line segment body is closest (this check works because the origin cant be past B)
        if (AB.dotProduct(AO) > 0) {

            if (AB.perpDotProduct(AO) > 0) // To the left of AB
                gjkInfo.dir = AB.getNormal(); // Then use the left normal
            else
                gjkInfo.dir = AB.getNormal().getNegated(); // Otherwise use the right normal

        }
        // Otherwise, point A is closest.
        else {
            gjkInfo.simplex.remove(0); // Remove B
            gjkInfo.dir = AO;
        }

        gjkInfo.isColliding = false;

    }

    /**
     * Compute the new search direction and new simplex if it is currently a
     * triangle. <br>
     * Like the line case, B, C, or BC cannot the closest features to the
     * origin. So they are automatically discarded and checks are not done.
     * 
     * @param gjkInfo the current state of the algorithm execution.
     */
    private void computeTriangleSimplex(SimplexDirStruct gjkInfo) {

        //simplex mapping: A=2, B=1, C=0

        /*
         * A is the newest point added. So we dont have to check edge BC because
         * the origin is not there. We also don't have to check B or C.
         */

        Vec2D AB, AC, AO;

        // The normal pointing outwards from the triangle.
        Vec2D ABOutNorm, ACOutNorm;

        /*
         * The following variable is required to correctly identify if the left or right normal of AB and AC are required.
         * That is, the modifier is applied to ABOutNorm and ACOutNorm.
         */
        int currentWindingModifier;

        AB = Vec2D.sub(gjkInfo.simplex.get(1), gjkInfo.simplex.get(2)); // B - A
        AC = Vec2D.sub(gjkInfo.simplex.get(0), gjkInfo.simplex.get(2)); // C - A

        AO = gjkInfo.simplex.get(2).getNegated();

        currentWindingModifier = calculateInitialWindingModifier(AB, gjkInfo.simplex.get(0));

        ABOutNorm = AB.getNormal();
        ABOutNorm.scaleBy(currentWindingModifier);

        double t = ABOutNorm.dotProduct(AO);

        // Somewhere outside AB (outside the triangle's boundary)
        if (t > 0) {

            // Somewhere past A's voronoi region, inside AB's voro region
            if (AB.dotProduct(AO) > 0) {
                gjkInfo.simplex.remove(0); // Remove C
                gjkInfo.dir = ABOutNorm;
                gjkInfo.isColliding = false;
                return;
            }
            // Inside A's voro region.
            else {
                gjkInfo.simplex.remove(1); // Remove B.
                gjkInfo.simplex.remove(0); // Remove C.
                gjkInfo.dir = AO;
                gjkInfo.isColliding = false;
                return;
            }

        }
        else if (MiscTools.tolEquals(t, 0)) { // Very close to AB.
            gjkInfo.simplex.remove(0); // Remove C
            gjkInfo.setDir(Vec2D.ORIGIN);
            gjkInfo.isColliding = false;
            return;
        }

        //Not in AB's line or A's voronoi region

        ACOutNorm = AC.getNormal();
        ACOutNorm.scaleBy(-currentWindingModifier);

        t = ACOutNorm.dotProduct(AO);

        // Somewhere outside AC (outside the triangle's boundary)
        if (t > 0) {

            // Somewhere past A's voro region, inside AC's voro region.
            if (AC.dotProduct(AO) > 0) {
                gjkInfo.simplex.remove(1); // Remove B
                gjkInfo.dir = ACOutNorm;
                gjkInfo.isColliding = false;
                return;
            }
            // Inside A's voronoi region.
            else {
                gjkInfo.simplex.remove(1); // Remove B.
                gjkInfo.simplex.remove(0); // Remove C.
                gjkInfo.dir = AO;
                gjkInfo.isColliding = false;
                return;
            }
        }
        else if (MiscTools.tolEquals(t, 0)) {
            gjkInfo.simplex.remove(1); // Remove B
            gjkInfo.setDir(Vec2D.ORIGIN);
            gjkInfo.isColliding = false;
            return;
        }

        // Because the point was not found outside either of the edges of the
        // triangle. Therefore, it must be inside the triangle.
        gjkInfo.isColliding = true;
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

    /**
     * Calculates the winding of the points of the current simplex.
     * 
     * @param AB any arbitrary edge of the triangular simplex.
     * @param C the vertex that is not included in the above edge.
     * @return the winding of the simplex ABC.
     */
    private int calculateInitialWindingModifier(Vec2D AB, Vec2D C) {

        if (AB.perpDotProduct(C) >= 0)
            return ANTICLOCKWISE_WINDING;

        return CLOCKWISE_WINDING;
    }

    /**
     * Returns the resolution vector if the two shapes are colliding, by using
     * the EPA algorithm. Otherwise, returns the minimum displacement between
     * the two shapes using GJK.<br>
     * <b>Colliding: </b>The returned vector is the displacement that needs to
     * be applied to s2 to completely unstick the shapes. <br>
     * <b>Not colliding: </b> The returned vector represents the amount that s1
     * needs to be displaced, to be in contact with s2. <br>
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @return the resolution vector if the shapes are colliding. The zero
     *         vector otherwise.
     */
    public SimplexDirStruct getCollisionResolution(Shape s1, Shape s2) {

        SimplexDirStruct gjkInfo = computeSimplex(s1, s2);

        if (gjkInfo.isColliding)
            computeCollisionResolutionEPA(s1, s2, gjkInfo);
        else
            computeMinimumDisplacement(s1, s2, gjkInfo);

        return gjkInfo;
    }

    /**
     * If a collision is not detected, returns the minimum displacement between
     * the two shapes. <br>
     * The displacement vector takes s2 as the reference shape. <br>
     * For example, if [-50,0] is returned, it means that s1 is 50 units to the
     * left of s2.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param gjkInfo the final simplex and search direction after GJK has run.
     */
    protected void computeMinimumDisplacement(Shape s1, Shape s2, SimplexDirStruct gjkInfo) {

        final double TOL = 0.1;

        if (gjkInfo.simplex.size() == 1) {
            gjkInfo.dir = gjkInfo.simplex.get(0).getNegated();
            return;
        }

        /*
         * If there's a triangle simplex, cull it down to a lower
         * dimensional simplex. There's no way for the simplex to contain the
         * origin. Because, you know, we wouldn't be here if it did.
         */
        else if (gjkInfo.simplex.size() == 3) {
            computeTriangleSimplex(gjkInfo);

            // If the simplex was originally a triangle, and then was evolved
            // into a point above, then we need to "re-evolve" it into a line.
            if (gjkInfo.simplex.size() == 1) {
                // The last search direction is unchecked and is still pointing
                // at the origin.
                gjkInfo.simplex.add(support(s1, s2, gjkInfo.dir));
            }
            // Now we have a line simplex and are ready to march.
        }
        // Next, start the march towards the origin till the tol is reached.
        // getCollisionResolution(s1, s2);
        while (true) {

            // Find closest point on line segment to the origin. Using
            // same protocol as previous simplex where latest point is A.
            int x = 0;
            Vec2D AB = Vec2D.sub(gjkInfo.simplex.get(0), gjkInfo.simplex.get(1)); //B - A
            Vec2D AO = gjkInfo.simplex.get(1).getNegated();
            Vec2D closestPt = AO.vecProjection(AB);

            closestPt.add(gjkInfo.simplex.get(1));

            // Now check if closestPt was outside of line seg.
            Vec2D ACl = Vec2D.sub(closestPt, gjkInfo.simplex.get(1));
            double ABdotACl = AB.dotProduct(ACl);

            if (ABdotACl <= 0) {
                closestPt = gjkInfo.simplex.get(1).getCopy();
            }
            else if (ABdotACl >= AB.getSquaredLength()) {
                closestPt = gjkInfo.simplex.get(0).getCopy();
            }

            if (closestPt.equals(Vec2D.ORIGIN)) {
                gjkInfo.dir = Vec2D.ORIGIN;
                return;
            }

            // Find the direction of the origin from the closest point.
            closestPt.negate();
            gjkInfo.dir = closestPt;

            Vec2D newPt;
            newPt = support(s1, s2, gjkInfo.dir);

            // Check if the new support is actually making progress towards the
            // origin.
            if (Vec2D.sub(newPt, gjkInfo.simplex.get(1)).dotProduct(gjkInfo.dir) <= TOL) { // If no progress
                gjkInfo.dir = closestPt;
                return;
            }

            // If progress was made, replace a bad point in the simplex with the
            // new support.
            if (gjkInfo.simplex.get(0).getSquaredLength() > gjkInfo.simplex
                    .get(1).getSquaredLength())
                gjkInfo.simplex.set(0, newPt);
            else
                gjkInfo.simplex.set(1, newPt);
        }
    }

    /**
     * Using the EPA algorithm, compute the collision normal and penetration
     * depth. <br>
     * Returns the vector by which s2 needs to be translated to be completely
     * removed from s1. <br>
     * For example, if [10,0] is returned, s2 needs to be moved 10 units to the
     * right to unstick the shapes.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param simplex the result simplex from running GJK on the shapes.
     * @return the collision normal between the two shapes.
     */
    protected void computeCollisionResolutionEPA(Shape s1, Shape s2,
            SimplexDirStruct gjkInfo) {

        // TODO try and make it so that we dont need this function call.
        gjkInfo.simplex = Polygon.arrangePoints(gjkInfo.simplex);

        final double TOL = 0.1;
        int count = 0;

        while (count < 20) {
            Vec2D[] closestEdge = null;

            double closestDist = Double.POSITIVE_INFINITY;

            int insertionIndex = 0;

            Vec2D[] edge;
            double dist;

            //Find closest edge to the origin.
            for (int i = 0; i < gjkInfo.simplex.size(); i++) {

                int j = (i + 1 == gjkInfo.simplex.size()) ? 0 : i + 1;

                edge = new Vec2D[] { gjkInfo.simplex.get(i), gjkInfo.simplex.get(j) };
                dist = LinePolyTools.ptToLineSegDisp(Vec2D.ORIGIN, edge).getSquaredLength();

                if (dist < closestDist) {
                    closestEdge = edge;
                    closestDist = dist;
                    insertionIndex = j;
                }
            }
            //end closest end finding. Now closestEdge and closestDist are valid.

            Vec2D edgeNorm = Vec2D.sub(closestEdge[1], closestEdge[0]).getNormal();
            edgeNorm.normalize();

            Vec2D newPt = support(s1, s2, edgeNorm);

            double newPtDistFromOrigin = Math.abs(newPt.dotProduct(edgeNorm));
            newPtDistFromOrigin *= newPtDistFromOrigin; // Square itself to get the squared dist.
                                                        // To match the scale of closestDist.

            if (newPtDistFromOrigin - closestDist <= TOL) {
                gjkInfo.dir = LinePolyTools.ptToLineDisp(Vec2D.ORIGIN, closestEdge);
                return;
            }
            else {
                gjkInfo.simplex.add(insertionIndex, newPt);
            }
            count++;
        }

        System.err.println("EPA v2 checker failure!");
        System.exit(1);
        gjkInfo.dir = Vec2D.ORIGIN;
        return;
    }

    /**
     * Using GJK, return whether the shapes s1 and s2 are colliding.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @return true if s1 and s2 intersect, false otherwise.
     */
    public boolean isColliding(Shape s1, Shape s2) {
        return computeSimplex(s1, s2).isColliding;
    }

    /**
     * Given a fully computed non-penetrating simplex, find the displacement
     * from the simplex to the origin, and set it as the direction. <br>
     * The simplex is assumed to be a point or a line. Under the assumption that
     * a non-penetrating simplex would never have 3 vertices. <br>
     * <i>Usage tip: </i> This method was more or less designed for use AFTER
     * <code>getCollisionResolution()</code> has already been run. Because, it
     * guarantees that the simplex conforms to the expected characteristics.
     * 
     * @param gjkInfo the structure containing the resultant information
     *            (simplex, direction, etc) from the collision test of some
     *            shapes.
     */
    public static void resetMinimumDisplacement(SimplexDirStruct gjkInfo) {
        if (gjkInfo.getSimplex().size() == 1)
            gjkInfo.setDir(gjkInfo.getSimplex().get(0));

        else {
            Vec2D AB = Vec2D.sub(gjkInfo.getSimplex().get(0), gjkInfo.getSimplex().get(1));
            Vec2D AO = gjkInfo.getSimplex().get(1).getNegated();

            gjkInfo.setDir(AO.vecProjection(AB));
        }

    }

}
