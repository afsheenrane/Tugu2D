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
     * Using GJK, return whether the shapes s1 and s2 are colliding.
     * 
     * @param s1 the first shape.
     * @param s2 the second shape.
     * @return true if s1 and s2 intersect, false otherwise.
     */
    public static boolean isColliding(Shape s1, Shape s2) {

        Vec2D newPt;
        GJKStruct gjkInfo = new GJKStruct();

        int count = 0;

        gjkInfo.dir = Vec2D.sub(s2.getCOM(), s1.getCOM());
        gjkInfo.simplex.add(support(s1, s2, gjkInfo.dir));

        gjkInfo.dir.negate();

        while (count < 20) {
            newPt = support(s1, s2, gjkInfo.dir);

            // If the new point is not past the origin, then the origin cannot
            // be encapsulated.
            if (newPt.dotProduct(gjkInfo.dir) < 0) {
                return false;
            }

            gjkInfo.simplex.add(newPt);

            if (computeSimplex(gjkInfo)) {
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
    private static boolean computeSimplex(GJKStruct gjkInfo) {

        switch (gjkInfo.simplex.size()) {
            case 2:
                return computeLineSimplex(gjkInfo);
            case 3:
                return computeTriangleSimplex(gjkInfo);
            default:
                System.err.println("Simplex size error: "
                        + gjkInfo.simplex.size());
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
    private static boolean computeLineSimplex(GJKStruct g) {

        // Line: B-------------A
        // B=0,A=1

        Vec2D AB, AO;

        AB = Vec2D.sub(g.simplex.get(0), g.simplex.get(1));
        AO = g.simplex.get(1).getNegated();

        // If the line segment body is closest:
        if (AB.dotProduct(AO) > 0) {

            if (AB.perpDotProduct(AO) > 0)
                g.dir = AB.getNormal();
            else
                g.dir = AB.getNormal().getNegated();

        }
        // Otherwise, point A is closest.
        else {
            g.simplex.remove(0); // Remove B
            g.dir = AO;
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
    private static boolean computeTriangleSimplex(GJKStruct g) {

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

        AB = Vec2D.sub(g.simplex.get(1), g.simplex.get(2));
        AO = g.simplex.get(2).getNegated();

        ABnorm = AB.getNormal().getNegated(); // Because we need the right norm.

        // Somewhere past AB
        if (ABnorm.dotProduct(AO) > 0) {

            // Somewhere past A's voronoi region, inside AB's voro region
            if (AB.dotProduct(AO) > 0) {
                g.simplex.remove(2); // Remove C
                g.dir = ABnorm;
                return false;
            }
            // Inside A's voro region.
            else {
                g.simplex.remove(0); // Remove C.
                g.simplex.remove(1); // Remove B.
                g.dir = AO;
                return false;
            }

        }

        AC = Vec2D.sub(g.simplex.get(0), g.simplex.get(2));
        ACnorm = AC.getNormal();

        // Somewhere past AC
        if (ACnorm.dotProduct(AO) > 0) {

            // Somewhere past A's voro region, inside AC's voro region.
            if (AC.dotProduct(AO) > 0) {
                g.simplex.remove(1); // Remove B
                g.dir = ACnorm;
                return false;
            }
            // Inside A's voronoi region.
            else {
                g.simplex.remove(0); // Remove C.
                g.simplex.remove(1); // Remove B.
                g.dir = AO;
                return false;
            }
        }

        // Because the point was not found outside either of the edges of the
        // triangle. Therefore, it must be inside the triangle.
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

/**
 * This class holds all the vital information used by th GJK algorithm while it
 * computes whether a collision has taken place.
 * 
 * @author Afsheen
 *
 */
class GJKStruct {

    /**
     * The simplex for the gjk algorithm.
     */
    ArrayList<Vec2D> simplex;

    /**
     * The current search direction.
     */
    Vec2D dir;

    /**
     * Initialize a new GJKStruct with an empty simplex of size 3 and a search
     * direction = [0,0].
     */
    GJKStruct() {
        this.simplex = new ArrayList<Vec2D>(3);
        this.dir = new Vec2D();
    }

}
