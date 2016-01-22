package phys2d.entities.shapes.polygons;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;

import phys2d.Phys2DMain;
import phys2d.collisionLogic.tools.MiscTools;
import phys2d.entities.Material;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;

public class Polygon extends Shape {

    protected double angularVelocity;

    protected Vec2D[] purePoly;

    /**
     * Create a new polygon consisting of 'points' vertices of material rubber.
     * 
     * @param points the vertices of the polygon.
     */
    public Polygon(Vec2D[] points) {
        this(points, 0);
    }

    /**
     * Create a new polygon defined by points. With no rotation and 10 units
     * mass.
     * 
     * @param points the edges of the unrotated polygon
     */
    public Polygon(Vec2D[] points, double angle) {
        this(points, angle, Material.RUBBER);
    }

    /**
     * Create a new polygon defined by points. With rotation of angle radians
     * and mass units of mass.
     * 
     * @param points the edges of the unrotated polygon
     * @param angle radians by which to rotate the polygon
     * @param mass mass of the polygon
     */
    public Polygon(Vec2D[] points, double angle, Material material) {
        this(points, null, angle, material);
    }

    /**
     * 
     * @param points the edges of the unrotated polygon
     * @param centerOfMass a precomputed center of mass of this polygon. (Also
     *            known as the centroid).
     * @param angle radians by which to rotate the polygon
     * @param mass mass of the polygon
     */
    public Polygon(Vec2D[] points, Vec2D centerOfMass, double angle, Material material) {
        super(points, centerOfMass, angle, 10);

        this.material = material;

        // Below, using the assumption that if a COM has already been allocated,
        // then the points must already be in a logical order.
        if (centerOfMass == null) {
            allocatePoints(points);
            calculateCOMAndArea();
        }
        else { //COM already allocated, so calc the area.
            this.area = Polygon.calculateArea(this.points);
        }

        setMass(area * material.getDensity());

        this.prevPos = new Vec2D[this.points.length + 1];

        resetPrevPos();

        lastAccel = new Vec2D(0, 0);

        // generate the pure poly before rotation is applied.
        this.purePoly = generatePurePoly(this.points, this.centerOfMass);

        if (angle != 0.0)
            setAngle(angle);
    }

    /**
     * Create a reference polygon which is used for rotation. This polygon is
     * centered on the origin and cannot be translated.
     * 
     * @return a reference, non-rotated polygon centered on the origin.
     */
    protected static Vec2D[] generatePurePoly(Vec2D[] points, Vec2D centerOfMass) {
        Vec2D[] t = new Vec2D[points.length];
        for (int i = 0; i < points.length; i++) {
            t[i] = Vec2D.sub(points[i], centerOfMass);
        }
        return t;
    }

    /**
     * Allocate all the entered points, clockwise into a convex polygon.
     * Starting at the coordinate with the lowest x value.
     */
    protected void allocatePoints(Vec2D[] points) {
        this.points = Polygon.arrangePoints(points);
    }

    /**
     * Calculates the area of the entered convex polygon.
     * 
     * @param points the vertices of the polygon in <b>CLOCKWISE</b> order.
     * @return the area of the entered polygon.
     */
    protected static double calculateArea(Vec2D[] points) {

        Vec2D p1, p2;
        double area = 0;

        for (int i = 0; i < points.length; i++) {

            p1 = points[i];
            p2 = (i + 1 == points.length) ? points[0] : points[i + 1];

            area += ((p1.getX() * p2.getY()) - (p2.getX() * p1.getY()));
        }

        area *= 0.5;

        area /= 10000.0; // Convert the area from px^2 (cm^2) to m^2

        area = Math.abs(area);

        return area;
    }

    /**
     * @param angle the angle to set.
     */
    @Override
    public void setAngle(double angle) {
        if (angle < 0)
            setAngle((Math.PI * 2) + angle);
        else {
            this.angle = angle % (2.0 * Math.PI);
            rotatePoints();
        }
    }

    /**
     * Rotate the points of the polygon about the COM to the current angle.
     */
    public void rotatePoints() {
        for (int i = 0; i < points.length; i++) {

            // No need to translate to origin because the purePoly is already centered on the origin.
            double pureX = purePoly[i].getX();
            double pureY = purePoly[i].getY();

            // rotate the point
            points[i].setX((pureX * Math.cos(angle)) - (pureY * Math.sin(angle)));
            points[i].setY((pureX * Math.sin(angle)) + (pureY * Math.cos(angle)));

            // Translate the point so that it is centered around center of mass
            points[i].add(centerOfMass);
        }
    }

    /**
     * Set the rotation speed in rads/s.
     * 
     * @param speed the angular velocity (rad/s)
     * @param updateRate the rate at which the physics engine updates. This is
     *            used to calculate the speed from px/s to px/update.
     */
    public void setConstantRotationSpeed(double speed, double updateRate) {
        this.angularVelocity = speed / updateRate;
    }

    /**
     * Return the minimum and maximum edges of the polygon along the ref axis.
     * 
     * @param ref the reference axis along which to measure
     * @return the minimum and maximum edges of the polygon (in that order) in
     *         reference to ref.
     */
    @Override
    public Vec2D[] getMinMax(Vec2D ref) {
        return MiscTools.getMinMax(points, ref);
    }

    /**
     * @param ref the reference vector along which to check for the minimum.
     * @return the minimum edge of this polygon along the ref vector.
     */
    @Override
    public Vec2D getMin(Vec2D ref) {
        return MiscTools.getMin(points, ref);
    }

    /**
     * @param ref the reference vector along which to check for the maximum.
     * @return the maximum edge of this polygon along the ref vector.
     */
    @Override
    public Vec2D getMax(Vec2D ref) {
        return MiscTools.getMax(points, ref);
    }

    /**
     * Return all the left normals of the polygon.
     * 
     * @return an array of vectors which are normals of the sides of the
     *         polygon.
     */
    public Vec2D[] getNormals() {
        Vec2D[] normals = new Vec2D[points.length];

        normals[normals.length - 1] = Vec2D.sub(points[0], points[points.length - 1]).getNormal();

        for (int i = 0; i < points.length - 1; i++) {
            normals[i] = Vec2D.sub(points[i + 1], points[i]).getNormal();
        }
        return normals;
    }

    /**
     * Shift the polygon by the translation vector.
     * 
     * @param translation the vector by which to shift the polygon.
     */
    @Override
    public void translate(Vec2D translation) {
        centerOfMass.add(translation);
        for (Vec2D p : points) {
            p.add(translation);
        }
    }

    /**
     * Apply all impending forces to this polygon and move this polygon
     * accordingly.
     * 
     * @param dt the physical delta time of the simulation.
     */
    @Override
    public void move(double dt) { // TODO

        resetPrevPos();

        // Symp EULER INTEGRATOR
        Vec2D accel = Vec2D.getScaled(netForce, invMass);
        velocity.add(accel);
        translate(Vec2D.getScaled(velocity, dt));

        netForce = new Vec2D(0, 0);

        if (angularVelocity != 0.0) // if there is a spin, apply it
            setAngle(angle + angularVelocity);
    }

    /**
     * Apply all impending forces to this polygon and move this polygon
     * forward the percent of dt specificed in the modifier.
     * 
     * @param dt the physical delta time of the simulation.
     * @param modifier the percent of the frame by which this polygon must be
     *            advanced.
     */
    @Override
    public void incrementMove(double dt, double modifier) {

        resetPrevPos();

        // Symp EULER INTEGRATOR
        Vec2D accel = Vec2D.getScaled(netForce, invMass);
        velocity.add(accel);
        translate(Vec2D.getScaled(velocity, dt).getScaled(modifier));

        netForce = new Vec2D(0, 0);
    }

    /**
     * Calculate the centerOfMass and area for this polygon given its current
     * points.
     */
    protected void calculateCOMAndArea() {
        double xSum = 0.0;
        double ySum = 0.0;
        double areaSum = 0.0;
        Vec2D p1, p2;

        for (int i = 0; i < points.length; i++) {

            p1 = points[i];
            p2 = (i + 1 == points.length) ? points[0] : points[i + 1];

            areaSum = ((p1.getX() * p2.getY()) - (p2.getX() * p1.getY()));

            xSum += (p1.getX() + p2.getX()) * areaSum;
            ySum += (p1.getY() + p2.getY()) * areaSum;

            area += areaSum;
        }

        area *= 0.5;

        area /= 10000.0; // convert from px^2 (cm^2) to m^2

        area = Math.abs(area);

        centerOfMass = new Vec2D(xSum / (6.0 * area), ySum / (6.0 * area));
    }

    /**
     * Resets the prevPos to the current pos. Presumably because the current pos
     * is about to be updated.
     */
    protected void resetPrevPos() {
        for (int i = 0; i < this.points.length; i++) {
            this.prevPos[i] = this.points[i].getCopy();
        }

        this.prevPos[this.prevPos.length - 1] = this.centerOfMass.getCopy();
    }

    /**
     * Sort the pts in clockwise order as if they are vertices of a polygon.
     * 
     * @param pts the points to sort.
     * @return the clockwise sorted array of points.
     */
    public static ArrayList<Vec2D> arrangePoints(ArrayList<Vec2D> pts) {

        if (pts.size() > 1) {

            ArrayList<Vec2D> convexPolyPts = new ArrayList<Vec2D>(pts.size());

            pts.sort((Vec2D v1, Vec2D v2) -> {
                if (v1.getX() < v2.getX())
                    return -1;
                else if (v1.getX() > v2.getX())
                    return 1;
                return 0;
            });

            //Now all pts should be sorted by x.

            Vec2D leftmost = pts.get(0);
            Vec2D rightmost = pts.get(pts.size() - 1);

            Vec2D divAxisNorm = Vec2D.sub(rightmost, leftmost).getNormal();

            // arraylist because the size will be more or less unknown
            ArrayList<Vec2D> upper = new ArrayList<Vec2D>(pts.size());
            ArrayList<Vec2D> lower = new ArrayList<Vec2D>(pts.size());

            Vec2D relVec;

            upper.add(leftmost); // I like my points clockwise

            for (Vec2D p : pts) { // foreach p in xSorted
                relVec = Vec2D.sub(p, leftmost);

                // using != instead of .equals because we only want to ignore exactly the left/right points. Not ignore dupes.
                if (p != leftmost && p != rightmost) {

                    // positive dot product of the normal => points above divider line
                    if (relVec.dotProduct(divAxisNorm) >= 0)
                        upper.add(p);
                    else
                        lower.add(p);
                }
            }

            lower.add(rightmost);

            // add upper regularly
            for (int i = 0; i < upper.size(); i++) {
                convexPolyPts.add(upper.get(i));
            }

            // lower needs to be added in reverse order
            int len = lower.size();
            for (int i = 0; i < lower.size(); i++) {
                convexPolyPts.add(lower.get(len - 1 - i));
            }

            return convexPolyPts;
        }
        else {
            return pts;
        }
    }

    /**
     * Sort the pts in clockwise order as if they are vertices of a polygon.
     * 
     * @param pts the points to sort.
     * @return the clockwise sorted array of points.
     */
    public static Vec2D[] arrangePoints(Vec2D[] pts) {

        if (pts.length > 1) {

            Vec2D[] convexPolyPts = new Vec2D[pts.length];

            // first sort the points by x values
            Vec2D[] xSorted = Vec2D.sortByX(pts); // is a standard ins sort right now

            // points of interest
            Vec2D leftmost = xSorted[0];
            Vec2D rightmost = xSorted[xSorted.length - 1];

            // do (rightmost - leftmost) to find the dividing line along which upper and lower points will be distributed
            Vec2D divAxisNorm = Vec2D.sub(rightmost, leftmost).getNormal();

            // arraylist because the size will be more or less unknown
            ArrayList<Vec2D> upper = new ArrayList<Vec2D>(pts.length);
            ArrayList<Vec2D> lower = new ArrayList<Vec2D>(pts.length);

            Vec2D relVec;

            upper.add(leftmost); // I like my points clockwise

            for (Vec2D p : xSorted) { // foreach p in xSorted
                relVec = Vec2D.sub(p, leftmost);

                // using != instead of .equals because we only want to ignore exactly the left/right points. Not ignore dupes.
                if (p != leftmost && p != rightmost) {

                    // positive dot product of the normal => points above divider line
                    if (relVec.dotProduct(divAxisNorm) >= 0)
                        upper.add(p);
                    else
                        lower.add(p);
                }
            }

            lower.add(rightmost);

            // add upper regularly
            for (int i = 0; i < upper.size(); i++) {
                convexPolyPts[i] = upper.get(i);
            }

            // lower needs to be added in reverse order
            int i = convexPolyPts.length - 1;
            for (Vec2D p : lower) {
                convexPolyPts[i] = p;
                i--;
            }

            return convexPolyPts;
        }
        else { //Only 1 pt...
            return pts;
        }
    }

    /**
     * @return the bounding AABB of this polygon in min-max form.
     */
    @Override
    public Vec2D[] getAABBbounds() {
        return Polygon.generateAABBbounds(points);
    }

    /**
     * Return the min and max points of the AABB from which the vertices can be
     * extracted.
     * 
     * @param pts
     * @return
     */
    public static Vec2D[] generateAABBbounds(Vec2D[] pts) {
        Vec2D[] xmm = MiscTools.getMinMax(pts, new Vec2D(1, 0));
        Vec2D[] ymm = MiscTools.getMinMax(pts, new Vec2D(0, 1));

        Vec2D[] aabbPts = new Vec2D[] {
                new Vec2D(xmm[0].getX(), ymm[0].getY()), // Min
                new Vec2D(xmm[1].getX(), ymm[1].getY()) // Max
        };

        return aabbPts;
    }

    /**
     * Return all the vertices of the aabb generated by the given points
     * 
     * @param pts
     * @return
     */
    public static Vec2D[] generateFullAABBbounds(Vec2D[] pts) {
        Vec2D[] xmm = MiscTools.getMinMax(pts, new Vec2D(1, 0));
        Vec2D[] ymm = MiscTools.getMinMax(pts, new Vec2D(0, 1));

        Vec2D[] aabbPts = new Vec2D[] {
                new Vec2D(xmm[0].getX(), ymm[0].getY()), // Min
                new Vec2D(xmm[0].getX(), ymm[1].getY()),
                new Vec2D(xmm[1].getX(), ymm[1].getY()), // Max
                new Vec2D(xmm[1].getX(), ymm[0].getY()) };

        return aabbPts;
    }

    /**
     * Return the min and max points of the shape's AABB swept along its
     * velocity vector for 1 update
     * 
     * @return
     */
    @Override
    public Vec2D[] getSweptAABBbounds(double dt) {

        Vec2D[] currentAABB = Polygon.generateAABBbounds(this.points); // get minmax aabb of this shape
        Vec2D[] translatedAABB = new Vec2D[2]; // the array which will store the velocity shifted minmax
        Vec2D[] allPts = new Vec2D[4]; // the points in the old and swept aabb will be kept here

        for (int i = 0; i < 2; i++) {
            // do the minkowski sum of the aabb and the velocity
            translatedAABB[i] = Vec2D.add(currentAABB[i], Vec2D.getScaled(this.velocity, dt));
            allPts[i] = currentAABB[i]; // just populating allpts
            allPts[currentAABB.length + i] = translatedAABB[i]; // same thing. just populating
        }

        return generateAABBbounds(allPts); // "generate a bunch of minmax's and let god sort out the rest"
    }

    /**
     * Draw the polygon with a center of mass
     * 
     * @param g2d
     * @param alpha
     * @param mode 0 = polygon with centroid and points. 1 = include AABB bounds
     *            2 = outline only with points of interest
     */
    public void draw(Graphics2D g2d, double alpha, int mode) {
        int[] xCoords = new int[points.length];
        int[] yCoords = new int[points.length];

        Color t = g2d.getColor();

        g2d.setColor(material.getColor());
        // allocate the points into arrays for use in the g2d methods
        for (int i = 0; i < points.length; i++) {
            xCoords[i] = (int) Math.round(((points[i].getX() * alpha) + (prevPos[i].getX() * (1.0 - alpha))));
            yCoords[i] = (int) Math.round(((points[i].getY() * alpha) + (prevPos[i].getY() * (1.0 - alpha))));

            yCoords[i] = Phys2DMain.YRES - yCoords[i];
        }

        // draw the polygon
        if (points.length == 2)
            g2d.drawLine(xCoords[0], yCoords[0], xCoords[1], yCoords[1]);

        else if (mode != 2) {
            g2d.fillPolygon(xCoords, yCoords, points.length); // full polygon
            g2d.setColor(Color.blue);
            g2d.drawPolygon(xCoords, yCoords, points.length); // outline
        }
        else
            g2d.drawPolygon(xCoords, yCoords, points.length); // outline

        // draw COM
        g2d.setColor(Color.RED);
        int xInterp, yInterp;

        xInterp = (int) Math.round(((centerOfMass.getX() * alpha) + (prevPos[prevPos.length - 1].getX() * (1.0 - alpha))));
        yInterp = (int) Math.round(((centerOfMass.getY() * alpha) + (prevPos[prevPos.length - 1].getY() * (1.0 - alpha))));

        yInterp = Phys2DMain.YRES - yInterp;

        //g2d.drawString(centerOfMass + "", xInterp - 35, yInterp - 13);
        g2d.fillOval(xInterp, yInterp, 3, 3);

        // if mode 1, draw AABB
        if (mode == 1) {
            Vec2D[] AABBPts = getAABBbounds();

            xCoords = new int[4];
            yCoords = new int[4];

            for (int i = 0; i < xCoords.length; i++) {
                xCoords[i] = (int) AABBPts[i].getX();
                yCoords[i] = (int) AABBPts[i].getY();

                yCoords[i] = Phys2DMain.YRES - yCoords[i];
            }
            g2d.drawPolygon(xCoords, yCoords, xCoords.length);
        }

        // reset the color
        g2d.setColor(t);
    }

    /**
     * draw a solid polygon with a labelled COM and points
     * 
     * @param g2d
     * @param alpha
     */
    @Override
    public void draw(Graphics2D g2d, double alpha) {
        draw(g2d, alpha, 0);
    }

    /**
     * draw a solid polygon with a bounding AABB with labelled COM and points
     * 
     * @param g2d
     * @param alpha
     */
    public void drawWithAABB(Graphics2D g2d, double alpha) {
        draw(g2d, alpha, 1);
    }

    public void drawOutline(Graphics2D g2d, double alpha) {
        draw(g2d, alpha, 2);
    }

    /**
     * Return a more or less accurate string representation of this polygon in
     * its current form.
     * 
     * @return The string representation of this polygon.
     */
    @Override
    public String repr() {

        String ptsRepr = "";
        for (Vec2D point : points) {
            ptsRepr += point.repr() + ", ";
        }

        ptsRepr = ptsRepr.substring(0, ptsRepr.length() - 2);

        return String.format("new Polygon(new Vec2D[]{%s}, %.2f);", ptsRepr,
                mass);
    }

}
