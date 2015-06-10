package phys2d.collisionLogic.collisionCheckers;

import java.util.ArrayList;

import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;

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
    public static SimplexDirStruct computeSimplex(Shape s1, Shape s2) {

        Vec2D newPt;
        SimplexDirStruct gjkInfo = new SimplexDirStruct();

        int count = 0;

        gjkInfo.dir = Vec2D.sub(s2.getCOM(), s1.getCOM());
        gjkInfo.simplex.add(support(s1, s2, gjkInfo.dir));

        gjkInfo.dir.negate();

        while (count < 20) {
            newPt = support(s1, s2, gjkInfo.dir);

            // If the new point is not past the origin, then the origin cannot
            // be encapsulated.
            if (newPt.dotProduct(gjkInfo.dir) < 0) {
                gjkInfo.isColliding = false;
                return gjkInfo;
            }

            gjkInfo.simplex.add(newPt);
            computeSimplex(gjkInfo);
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
    private static void computeSimplex(SimplexDirStruct gjkInfo) {

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
        ;
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
    public Vec2D getCollisionResolution(Shape s1, Shape s2) {
        SimplexDirStruct gjkInfo = computeSimplex(s1, s2);

        if (!gjkInfo.isColliding) {
            return Vec2D.ORIGIN;
        }
        else {
            return computeCollisionResolutionEPA(s1, s2, gjkInfo.simplex);
        }

    }

    /**
     * 
     * @param s1
     * @param s2
     * @param simplex
     * @return
     */
    private Vec2D computeCollisionResolutionEPA(Shape s1, Shape s2,
            ArrayList<Vec2D> simplex) {

        return null;
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
