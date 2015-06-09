package phys2d.collisionLogic.collisionCheckers;

import java.util.ArrayList;

import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;

/**
 * A better written version of GJK and EPA with a focus on speed.
 * 
 * @author afsheen
 *
 */
public final class CollisionCheckerGJKEPA2 {

    /**
     * Using GJK, return whether the shapes s1 and s2 are colliding.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @return true if s1 and s2 intersect, false otherwise.
     */
    public static boolean isColliding(Shape s1, Shape s2) {

        Vec2D dir = Vec2D.sub(s2.getCOM(), s1.getCOM());
        Vec2D newPt;

        ArrayList<Vec2D> simplex = new ArrayList<>(3);

        int count = 0;

        simplex.add(support(s1, s2, dir));

        dir.negate();

        while (count < 20) {
            newPt = support(s1, s2, dir);

            // If the new point is not past the origin, then the origin cannot
            // be encapsulated.
            if (newPt.dotProduct(dir) < 0) {
                return false;
            }

            simplex.add(newPt);

            if (computeSimplex(simplex, dir)) {
                return true;
            }

            count++;

        }

        System.err.println("GJK TOL failure.");
        return false;

    }

    /**
     * Modifies the simplex according to it's current characteristics and change
     * the search direction if needed.
     * 
     * @param simplex the simplex computed so far.
     * @param dir the search direction.
     * @return true if the origin is inside the simplex, false otherwise.
     */
    private static boolean computeSimplex(ArrayList<Vec2D> simplex, Vec2D dir) {

        switch (simplex.size()) {
            case 2:
                return computeLineSimplex(simplex, dir);
            case 3:
                return computeTriangleSimplex(simplex, dir);
            default:
                System.err.println("Simplex size error: " + simplex.size());
                System.exit(0);
        }

        return false;
    }

    /**
     * Compute the new search direction and new simplex if it is currently a
     * line. <br>
     * The first point cannot be the closest feature to the origin. This is
     * because it was already deduced that the newly added point is in the
     * direction of the origin. Therefore, we only need to check if the new
     * point is closest to the origin, or whether the line body is closest.
     * 
     * @param simplex the simplex computed thus far.
     * @param dir the current search direction.
     * @return false because it is not possible to enclose the origin with only
     *         two points in R2.
     */
    private static boolean computeLineSimplex(ArrayList<Vec2D> simplex,
            Vec2D dir) {

        // B-------------A
        // B=0,A=1

        Vec2D AB, AO;

        AB = Vec2D.sub(simplex.get(0), simplex.get(1));
        AO = simplex.get(1).getNegated();

        // If the line segment body is closest:
        if (AB.dotProduct(AO) > 0) {

            if (AB.perpDotProduct(AO) > 0)
                dir = AB.getNormal(); // TODO mutation is not maintained here.
            else
                dir = AB.getNormal().getNegated();

        }
        // Otherwise, point A is closest.
        else {
            simplex.remove(0); // Remove B
            dir = AO;
        }

        return false;
    }

    /**
     * Compute the new search direction and new simplex if it is currently a
     * triangle. <br>
     * Like the line case, B, C, or BC cannot the closest features to the
     * origin. So they are automatically discarded and checks are not done.
     * 
     * @param simplex the simplex computed thus far.
     * @param dir the current search direction.
     * @return true if the origin is contained inside the triangle. False
     *         otherwise.
     */
    private static boolean computeTriangleSimplex(ArrayList<Vec2D> simplex,
            Vec2D dir) {

        //@formatter:off 
        
        /* 
         * Triangle:
         * ....A....
         * .../.\...
         * ../...\..
         * .B_____C.
         * 
         * simplex mapping: A=2, B=1, C=0
         */ 
        
        //@formatter:on

        /*
         * A is the newest point added. So we dont have to check edge BC
         * because the origin is not there. We also don't have to check B or C.
         */

        Vec2D AB, AC, AO;

        // The normal pointing outwards from the triangle.
        Vec2D ABnorm, ACnorm;

        AB = Vec2D.sub(simplex.get(1), simplex.get(2));
        AO = simplex.get(2).getNegated();

        ABnorm = AB.getNormal().getNegated(); // Because we need the right norm.

        // Somewhere past AB
        if (ABnorm.dotProduct(AO) > 0) {

            // Somewhere past A's voronoi region, inside AB's voro region
            if (AB.dotProduct(AO) > 0) {
                simplex.remove(2); // Remove C
                dir = ABnorm;
                return false;
            }
            // Inside A's voro region.
            else {
                simplex.remove(0); // Remove C.
                simplex.remove(1); // Remove B.
                dir = AO;
                return false;
            }

        }

        AC = Vec2D.sub(simplex.get(0), simplex.get(2));
        ACnorm = AC.getNormal();

        // Somewhere past AC
        if (ACnorm.dotProduct(AO) > 0) {

            // Somewhere past A's voro region, inside AC's voro region.
            if (AC.dotProduct(AO) > 0) {
                simplex.remove(1); // Remove B
                dir = ACnorm;
                return false;
            }
            // Inside A's voronoi region.
            else {
                simplex.remove(0); // Remove C.
                simplex.remove(1); // Remove B.
                dir = AO;
                return false;
            }
        }
        // Because
        return true;
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

}
