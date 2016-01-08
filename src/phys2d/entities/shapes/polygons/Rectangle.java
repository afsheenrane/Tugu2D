package phys2d.entities.shapes.polygons;

import phys2d.entities.Material;
import phys2d.entities.Symmetrical;
import phys2d.entities.Vec2D;

public class Rectangle extends Polygon implements Symmetrical {

    protected double length, height;

    /**
     * Initialize a rectangle with the center pos, length, height, and mass.
     * with a velocity of [0,0]
     * 
     * @param center the centroid of the rectangle
     * @param length horizontal extent of the rectangle
     * @param height vertical extent of the rectangle
     * @param angle the angle the rectangle is rotated
     * @param mass of the rectangle
     */
    public Rectangle(Vec2D center, double length, double height, double angle, Material material) {
        super(new Vec2D[] {
                new Vec2D(center.getX() - (length / 2.0), center.getY() + (height / 2.0)),
                new Vec2D(center.getX() + length / 2.0, center.getY() + (height / 2.0)),
                new Vec2D(center.getX() + length / 2.0, center.getY() - (height / 2.0)),
                new Vec2D(center.getX() - length / 2.0, center.getY() - (height / 2.0))
        },
                center,
                angle, material);
        this.length = length;
        this.height = height;
    }

    public Rectangle(Vec2D center, double length, double height) {
        this(center, length, height, 0.0, Material.RUBBER);
    }

    /**
     * Set the center of the rectangle
     * 
     * @param center the center of mass to set
     */
    public void setCenterOfMass(Vec2D center) {
        this.centerOfMass = center;
        points[0] = new Vec2D(center.getX() - (length / 2.0), center.getY() + (height / 2.0));
        points[1] = new Vec2D(center.getX() + length / 2.0, center.getY() + (height / 2.0));
        points[2] = new Vec2D(center.getX() + length / 2.0, center.getY() - (height / 2.0));
        points[3] = new Vec2D(center.getX() - length / 2.0, center.getY() - (height / 2.0));
    }

    /**
     * @return the length
     */
    public double getLength() {
        return length;
    }

    /**
     * @param length the length to set
     */
    public void setLength(double length) {
        if (length > 0) {
            this.length = length;
            setCenterOfMass(centerOfMass);
        }
        else {
            System.out.println("CANNOT HAVE LENGTH OF ZERO OR LESS!");
        }
    }

    /**
     * @return the height
     */
    public double getHeight() {
        return height;
    }

    /**
     * @param height the height to set
     */
    public void setHeight(double height) {
        if (height > 0) {
            this.height = height;
            setCenterOfMass(centerOfMass);
            purePoly = Polygon.generatePurePoly(this.points, this.centerOfMass);
        }
        else {
            System.out.println("CANNOT HAVE HEIGHT OF ZERO OR LESS!");
        }
    }

    @Override
    public Vec2D[] getUniqueNormals() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public String toString() {
        String pts = "[";
        for (Vec2D pt : getPoints()) {
            pts += pt + " ";
        }
        pts += "]";
        return String.format("%s: %s COM: %s length: %s  height: %s", getClass().getSimpleName(), pts, getCOM(), length, height);
    }

    /**
     * Return all the left normals which are on a unique axis
     * 
     * @return an array of 2 normal vectors of the square sides which do not
     *         share the same axis
     */
    /*
    @Override
    public Vec2D[] getUniqueNormals(){
    	Vec2D[] normals = new Vec2D[2];
    	for (int i = 0; i < 2; i++){
    		normals[i] = Vec2D.sub(points[i + 1], points[i]).getNormal();
    	}
    	return normals;
    }*/
}
