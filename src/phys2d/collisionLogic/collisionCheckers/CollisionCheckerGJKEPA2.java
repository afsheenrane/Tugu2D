package phys2d.collisionLogic.collisionCheckers;

import phys2d.collisionLogic.tools.LinePolyTools;
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

    /**
     * Using GJK, return the result of the algorithm on the two shapes.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @return a structure containing the final state of the simplex, the last
     *         search direction, and whether the shapes are colliding or not.
     */
    private static SimplexDirStruct computeSimplex(Shape s1, Shape s2) {

        Vec2D newPt;
        SimplexDirStruct gjkInfo = new SimplexDirStruct();

        int count = 0;

        gjkInfo.dir = Vec2D.sub(s2.getCOM(), s1.getCOM());
        gjkInfo.simplex.add(support(s1, s2, gjkInfo.dir));

        gjkInfo.dir.negate();

        while (count < 20) {
            newPt = support(s1, s2, gjkInfo.dir);
            gjkInfo.simplex.add(newPt);

            // If the new point is not past the origin, then the origin cannot
            // be encapsulated.
            if (newPt.dotProduct(gjkInfo.dir) < 0) {
                gjkInfo.isColliding = false;
                return gjkInfo;
            }

            evolveSimplex(gjkInfo);
            if (gjkInfo.isColliding) {
                return gjkInfo;
            }

            count++;

        }

        System.err.println("GJK TOL failure.");
        gjkInfo.isColliding = false;
        return gjkInfo;

    }

    /**
     * Modifies the simplex according to it's current characteristics and change
     * the search direction if needed.
     * 
     * @param gjkInfo the current state of the algorithm.
     */
    private static void evolveSimplex(SimplexDirStruct gjkInfo) {

        switch (gjkInfo.simplex.size()) {
            case 2:
                computeLineSimplex(gjkInfo);
                break;
            case 3:
                computeTriangleSimplex(gjkInfo);
                break;
            default:
                System.err.println("Simplex size error: "
                        + gjkInfo.simplex.size());

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
    private static void computeLineSimplex(SimplexDirStruct gjkInfo) {

        // Line: B-------------A
        // B=0,A=1

        Vec2D AB, AO;

        AB = Vec2D.sub(gjkInfo.simplex.get(0), gjkInfo.simplex.get(1));
        AO = gjkInfo.simplex.get(1).getNegated();

        // If the line segment body is closest:
        if (AB.dotProduct(AO) > 0) {

            if (AB.perpDotProduct(AO) > 0)
                gjkInfo.dir = AB.getNormal();
            else
                gjkInfo.dir = AB.getNormal().getNegated();

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
    private static void computeTriangleSimplex(SimplexDirStruct gjkInfo) {

        // @formatter:off
        /*
         * Triangle:
         * ....A....
         * .../.\...
         * ../...\..
         * .B_____C.
         * 
         * simplex mapping: A=2, B=1, C=0
         */
        // @formatter:on

        /*
         * A is the newest point added. So we dont have to check edge BC because
         * the origin is not there. We also don't have to check B or C.
         */

        Vec2D AB, AC, AO;

        // The normal pointing outwards from the triangle.
        Vec2D ABnorm, ACnorm;

        AB = Vec2D.sub(gjkInfo.simplex.get(1), gjkInfo.simplex.get(2));
        AO = gjkInfo.simplex.get(2).getNegated();

        ABnorm = AB.getNormal().getNegated(); // Because we need the right norm.

        // Somewhere past AB
        if (ABnorm.dotProduct(AO) > 0) {

            // Somewhere past A's voronoi region, inside AB's voro region
            if (AB.dotProduct(AO) > 0) {
                gjkInfo.simplex.remove(2); // Remove C
                gjkInfo.dir = ABnorm;
                gjkInfo.isColliding = false;
                return;
            }
            // Inside A's voro region.
            else {
                gjkInfo.simplex.remove(0); // Remove C.
                gjkInfo.simplex.remove(1); // Remove B.
                gjkInfo.dir = AO;
                gjkInfo.isColliding = false;
                return;
            }

        }

        AC = Vec2D.sub(gjkInfo.simplex.get(0), gjkInfo.simplex.get(2));
        ACnorm = AC.getNormal();

        // Somewhere past AC
        if (ACnorm.dotProduct(AO) > 0) {

            // Somewhere past A's voro region, inside AC's voro region.
            if (AC.dotProduct(AO) > 0) {
                gjkInfo.simplex.remove(1); // Remove B
                gjkInfo.dir = ACnorm;
                gjkInfo.isColliding = false;
                return;
            }
            // Inside A's voronoi region.
            else {
                gjkInfo.simplex.remove(0); // Remove C.
                gjkInfo.simplex.remove(1); // Remove B.
                gjkInfo.dir = AO;
                gjkInfo.isColliding = false;
                return;
            }
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
    private static Vec2D support(Shape s1, Shape s2, Vec2D dir) {
        return (Vec2D.sub(s1.getMax(dir), s2.getMin(dir)));
    }

    /**
     * Returns the resolution vector if the two shapes are colliding, by using
     * the EPA algorithm.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @return the resolution vector if the shapes are colliding. The zero
     *         vector otherwise.
     */
    public static SimplexDirStruct getCollisionResolution(Shape s1, Shape s2) {
        SimplexDirStruct gjkInfo = computeSimplex(s1, s2);

        if (!gjkInfo.isColliding) {
            // TODO return the minimum distance between the two shapes.
            return computeMinimumDisplacement(s1, s2, gjkInfo);
        }
        else {
            computeCollisionResolutionEPA(s1, s2, gjkInfo);
        }
        return gjkInfo;
    }

    /**
     * If a collision is not detected, returns the minimum displacement between
     * the two shapes.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param gjkInfo the final simplex and search direction after GJK has run.
     */
    private static void computeMinimumDisplacement(Shape s1, Shape s2,
            SimplexDirStruct gjkInfo) {
        // TODO finish this.

        final double TOL = 0.1;

        /*
         * First, if there's a triangle simplex, cull it down to a lower
         * dimensional simplex. There's no way for the simplex to contain the
         * origin. Because, you know, we wouldn't be here if it did.
         */
        if (gjkInfo.simplex.size() == 3)
            computeTriangleSimplex(gjkInfo);

        // Next, start the march towards the origin till the tol is reached.

        // If the simplex was originally a triangle, and then was evolved into a
        // point above, then we need to "re-evolve" it into a line.
        if (gjkInfo.simplex.size() == 1) {
            // The last search direction is unchecked and is still pointing at
            // the origin.
            gjkInfo.simplex.add(support(s1, s2, gjkInfo.dir));
        }

        // Now we have a line simplex and are ready to march.

        // Find closest point on line to the origin. Using same protocol as
        // previous simplex where latest point is A.
        Vec2D AB = Vec2D.sub(gjkInfo.simplex.get(0), gjkInfo.simplex.get(1));
        Vec2D AO = gjkInfo.simplex.get(1).getNegated();
        Vec2D closestPt = AO.vecProjection(AB);

        if (closestPt.equals(Vec2D.ORIGIN)) {
            gjkInfo.dir = Vec2D.ORIGIN;
            return;
        }

        Vec2D newPt;

        // Find the direction of the origin from the closest point.
        closestPt.negate();
        gjkInfo.dir = closestPt;

        newPt = support(s1, s2, gjkInfo.dir);

        // Check if the new support is actually making progress towards the
        // origin.
        if (Vec2D.sub(newPt, gjkInfo.simplex.get(1)).dotProduct(gjkInfo.dir) <= TOL) {
            gjkInfo.dir = closestPt;
            return;
        }

        // If progress was made, replace a bad point in the simplex with the new
        // support.
        if (gjkInfo.simplex.get(0).getSquaredLength() > gjkInfo.simplex.get(1)
                .getSquaredLength())
            gjkInfo.simplex.set(0, newPt);
        else
            gjkInfo.simplex.set(1, newPt);
    }

    /**
     * Using the EPA algorithm, compute the collision normal and penetration
     * depth.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @param simplex the result simplex from running GJK on the shapes.
     * @return the collision normal between the two shapes.
     */
    private static void computeCollisionResolutionEPA(Shape s1, Shape s2,
            SimplexDirStruct gjkInfo) {

        gjkInfo.simplex = Polygon.arrangePoints(gjkInfo.simplex);

        final double TOL = 0.1;
        int count = 0;

        while (count < 20) {
            Vec2D[] closestEdge = new Vec2D[] {
                    gjkInfo.simplex.get(gjkInfo.simplex.size() - 1),
                    gjkInfo.simplex.get(0) };

            double closestDist = computeRelativeDist(Vec2D.ORIGIN, closestEdge);
            int insertionIndex = 0;

            Vec2D[] edge;
            double dist;
            // 2 => simplex.size - 1
            for (int i = 0; i < 2; i++) {
                edge = new Vec2D[] { gjkInfo.simplex.get(i),
                        gjkInfo.simplex.get(i + 1) };
                dist = computeRelativeDist(Vec2D.ORIGIN, edge);

                if (dist < closestDist) {
                    closestEdge = edge;
                    closestDist = dist;
                    insertionIndex = i + 1;
                }
            }

            Vec2D edgeNorm = Vec2D.sub(closestEdge[1], closestEdge[0])
                    .getNormal();
            Vec2D newPt = support(s1, s2, edgeNorm);

            double newPtDistFromOrigin = Math.abs(newPt.dotProduct(edgeNorm));

            if (newPtDistFromOrigin - closestDist <= TOL) {
                gjkInfo.dir = LinePolyTools.ptToLineDisp(Vec2D.ORIGIN,
                        closestEdge);
                return;
            }
            else {
                gjkInfo.simplex.add(insertionIndex, newPt);
            }
            count++;
        }

        System.err.println("EPA v2 checker failure!");

        gjkInfo.dir = Vec2D.ORIGIN;
        return;
    }

    /**
     * Compute the relative displacement of p from the line.
     * 
     * @param p the point to check.
     * @param line the line to check p's displacement from.
     * @return a number directly proportional to the point's displacement from
     *         the line. <b> This is not an actual displacement.</b>
     */
    private static double computeRelativeDist(Vec2D p, Vec2D[] line) {
        Vec2D lineNorm = Vec2D.sub(line[1], line[0]).getNormal();
        Vec2D relVec = Vec2D.sub(p, line[0]);

        double dist = relVec.dotProduct(lineNorm);

        return Math.abs(dist);
    }

    /**
     * Using GJK, return whether the shapes s1 and s2 are colliding.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @return true if s1 and s2 intersect, false otherwise.
     */
    public static boolean isColliding(Shape s1, Shape s2) {
        return computeSimplex(s1, s2).isColliding;
    }

}
