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
public final class CollisionCheckerMPR {

    public boolean isColliding(Shape s1, Shape s2) {

        //@formatter:off
        /*
         * A---p>--B
         * .\...../.
         * ..1...2..
         * ...\./...
         * ....R....
         */
        //@formatter:on

        SimplexDirStruct mprInfo = new SimplexDirStruct();
        Vec2D ref = Vec2D.sub(s2.getCOM(), s1.getCOM()); // The main reference
                                                         // point. B-A
        Vec2D side1, side2, portal, side1norm;

        mprInfo.dir = ref.getNegated();
        mprInfo.simplex.add(support(s1, s2, mprInfo.dir)); // A

        side1 = Vec2D.sub(mprInfo.simplex.get(0), ref); // RA
        side1norm = side1.getNormal();

        // Find which side origin is on.
        if (side1norm.dotProduct(mprInfo.dir) > 0) {
            mprInfo.dir = side1norm;
        }
        else {
            mprInfo.dir = side1norm.getNegated();
        }

        mprInfo.simplex.add(support(s1, s2, mprInfo.dir)); // B

        portal = Vec2D.sub(mprInfo.simplex.get(1), mprInfo.simplex.get(0)); // AB

        // REFINEMENT PHASE. LOOP HERE?
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
