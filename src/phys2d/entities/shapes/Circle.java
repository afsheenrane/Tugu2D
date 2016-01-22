package phys2d.entities.shapes;

import java.awt.Color;
import java.awt.Graphics2D;

import phys2d.Phys2DMain;
import phys2d.entities.Material;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.polygons.Polygon;

public class Circle extends Shape {

    private double radius;

    /**
     * Default the mass to 10
     * 
     * @param pos the center of the circle
     * @param radius the radius of the circle
     */
    public Circle(Vec2D[] pos, double radius) {
        this(pos, radius, Material.RUBBER);
    }

    /**
     * 
     * @param pos the center of the circle
     * @param radius the radius of the circle
     */
    public Circle(Vec2D pos, double radius) {
        this(new Vec2D[] { pos }, radius, Material.RUBBER);
    }

    /**
     * @param pos the position of the center of the circle
     * @param radius the radius of the circle
     * @param mass the mass of the circle
     */
    public Circle(Vec2D[] pos, double radius, Material material) {
        super(pos, pos[0], 0, 10);
        this.prevPos = new Vec2D[1];
        prevPos[0] = pos[0];
        this.material = material;
        double temp = Math.PI * (radius * radius);
        temp /= 10000.0; // unit conversion cm^2 -> m^2
        setArea(temp);
        setMass(this.area * material.getDensity());
        this.radius = radius;
    }

    /**
     * Move the circle with the velocity vel
     * 
     * @param vel (px/s) the velocity to move the circle with
     */
    @Override
    public void move(double dt) {

        prevPos[0] = this.centerOfMass.getCopy();

        // Symplectic euler integrator
        Vec2D accel = Vec2D.getScaled(netForce, invMass);
        velocity.add(accel);
        translate(Vec2D.getScaled(velocity, dt));

        netForce = new Vec2D(0, 0);

    }

    @Override
    public void incrementMove(double dt, double modifier) {
        prevPos[0] = this.centerOfMass;

        // Symplectic euler integrator
        Vec2D accel = Vec2D.getScaled(netForce, invMass);
        velocity.add(accel);
        translate(Vec2D.getScaled(velocity, dt).getScaled(modifier));

        netForce = new Vec2D(0, 0);
    }

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    @Override
    public Vec2D[] getMinMax(Vec2D axis) {
        return new Vec2D[] { getMin(axis), getMax(axis) };
    }

    @Override
    public Vec2D getMin(Vec2D axis) {
        return getMax(Vec2D.getNegated(axis));
    }

    @Override
    public Vec2D getMax(Vec2D axis) {

        Vec2D max;

        axis = axis.getNormalized();
        axis.scaleBy(radius);
        max = Vec2D.add(points[0], axis);

        return max;
    }

    @Override
    public Vec2D[] getAABBbounds() {
        return new Vec2D[] {
                new Vec2D(points[0].getX() - radius, points[0].getY() - radius), // bottom left
                new Vec2D(points[0].getX() + radius, points[0].getY() + radius), // top right
        };
    }

    @Override
    public Vec2D[] getSweptAABBbounds(double dt) {
        Vec2D[] currentAABB = getAABBbounds();
        Vec2D[] translatedAABB = new Vec2D[2];
        Vec2D[] allPts = new Vec2D[4];

        for (int i = 0; i < 2; i++) {
            // do the minkowski sum of the aabb and the velocity
            translatedAABB[i] = Vec2D.add(currentAABB[i], this.velocity);
            allPts[i] = currentAABB[i];
            allPts[currentAABB.length + i] = translatedAABB[i];
        }
        return Polygon.arrangePoints(Polygon.generateAABBbounds(allPts));
    }

    @Override
    public void translate(Vec2D translation) {
        points[0].add(translation);
    }

    @Override
    public String repr() {

        return String.format("new Circle(new Vec2D(%s), %.2f);",
                points[0].getX() + ", " + points[0].getY(), radius);
    }

    @Override
    public void draw(Graphics2D g2d, double alpha) {
        Color t = g2d.getColor();

        // draw inner circle
        g2d.setColor(material.getColor());

        int xInterp, yInterp; // The interpolated positions

        xInterp = (int) Math.round(((points[0].getX() - radius) * alpha) + ((prevPos[0].getX() - radius) * (1.0 - alpha)));
        yInterp = (int) Math.round(((points[0].getY() + radius) * alpha) + ((prevPos[0].getY() + radius) * (1.0 - alpha)));
        yInterp = Phys2DMain.YRES - yInterp;

        // Inner circle
        g2d.fillOval(xInterp, yInterp, (int) (radius * 2.0), (int) (radius * 2.0));

        // outer border
        g2d.setColor(Color.blue);
        g2d.drawOval(xInterp, yInterp, (int) (radius * 2.0), (int) (radius * 2.0));

        // Draw COM
        g2d.setColor(Color.RED);
        //g2d.drawString(centerOfMass + "", (int) centerOfMass.getX() - 35, Phys2DMain.YRES - (int) centerOfMass.getY() - 13);

        g2d.fillOval(
                (int) Math.round(((points[0].getX() - 1) * alpha) + ((prevPos[0].getX() - 1) * (1.0 - alpha))),
                Phys2DMain.YRES - (int) Math.round(((points[0].getY() - 1) * alpha) + ((prevPos[0].getY() - 1) * (1.0 - alpha))),
                3, 3);

        g2d.setColor(t);
    }

    /**
     * Draw this circle with lines representing it maximum and minimum extents
     * in the x and y axes.
     * 
     * @param g2d
     * @param delta
     */
    public void drawWithGuides(Graphics2D g2d, double delta) {
        draw(g2d, delta);

        g2d.drawLine((int) (points[0].getX() - radius), 0, (int) (points[0].getX() - radius), 1000);
        g2d.drawLine((int) (points[0].getX() + radius), 0, (int) (points[0].getX() + radius), 1000);

        g2d.drawLine(0, (int) (points[0].getY() - radius), 1000, (int) (points[0].getY() - radius));
        g2d.drawLine(0, (int) (points[0].getY() + radius), 1000, (int) (points[0].getY() + radius));
    }

    @Override
    public String toString() {
        return "Circle: " + points[0] + " Radius: " + radius + " Velocity: " + velocity;
    }
}
