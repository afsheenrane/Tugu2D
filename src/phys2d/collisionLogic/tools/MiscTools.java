package phys2d.collisionLogic.tools;

import java.util.Random;

import phys2d.entities.Vec2D;

public final class MiscTools {

    /**
     * From the set of points return the minimum and maximum points along the
     * ref axis
     * 
     * @param points the set of points to check
     * @param ref the reference axis along which to check for minmax
     * @return an array of the min and max points (indices 0, 1 respectively) of
     *         the point set along the reference axis.
     */
    public static Vec2D[] getMinMax(Vec2D[] points, Vec2D ref) {
        return new Vec2D[] { getMin(points, ref), getMax(points, ref) };
    }

    public static Vec2D getMin(Vec2D[] points, Vec2D ref) {
        Vec2D min = points[0];
        Vec2D refNorm = ref.getNormal();

        double minDot = min.dotProduct(ref);
        double refDot;

        for (Vec2D p : points) {
            refDot = p.dotProduct(ref);

            if (refDot < minDot) {
                minDot = refDot;
                min = p;
            }

            /*
             * if they have the same projection along the reference axis, sort
             * them
             * by their projection in the reference normal.
             * eg: If ref is the x-axis, then the min vector will be the vector
             * with
             * the lowest x and y extents.
             */
            else if (refDot == minDot) {
                if (min.dotProduct(refNorm) > p.dotProduct(refNorm))
                    min = p; // no need to make minDot = refDot, because they're
                             // already equal.
            }
        }
        return min;
    }

    public static Vec2D getMax(Vec2D[] points, Vec2D ref) {
        return getMin(points, Vec2D.getNegated(ref));
    }

    public static Vec2D[] genRandVecs(int count, Vec2D floor, Vec2D ceiling) {
        return genRandVecs(count, floor, ceiling, new Random().nextLong());
    }

    /**
     * Generate a number of random vectors within the floor and ceiling
     * parameters.
     * 
     * @param count the number of vectors to generate.
     * @param floor the lowest x and y values that can be generated.
     * @param ceiling the highest x and y values that can be generated.
     * @param seed the seed to use the for random generator.
     * @return a vector array of size count containing randomly generated.
     *         vectors within the giving floor and ceiling parameters.
     */
    public static Vec2D[] genRandVecs(int count, Vec2D floor, Vec2D ceiling, long seed) {
        System.out.println("seed: " + seed);
        Random randGen = new Random(seed);
        Vec2D[] results = new Vec2D[count];

        double rx, ry;

        for (int i = 0; i < count; i++) {
            rx = randGen.nextInt((int) (ceiling.getX() - floor.getX()))
                    + floor.getX();
            rx += randGen.nextDouble();

            ry = randGen.nextInt((int) (ceiling.getY() - floor.getY()))
                    + floor.getY();
            ry += randGen.nextDouble();

            results[i] = new Vec2D(rx, ry);
        }
        return results;
    }

    /**
     * Round the specified number to the entered decimal places.
     * 
     * @param num
     * @param dec
     * @return
     */
    public static double roundToDecimal(double num, int dec) {
        double x = Math.pow(10, dec);

        num *= x;
        num = Math.round(num);
        num /= x;

        return num;
    }

    /**
     * Checks whether two doubles are equal to each other given a specific
     * tolerance. (0.001) in this case.
     * 
     * @param t1 the first double.
     * @param t2 the second double.
     * @return whether t1 almost equals t2.
     */
    public static boolean tolEquals(double t1, double t2) {
        final double TOL = 0.001;
        return Math.abs(t1 - t2) < TOL;
    }
}
