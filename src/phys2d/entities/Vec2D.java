package phys2d.entities;

import java.awt.Graphics2D;

public class Vec2D {

    public static final Vec2D ORIGIN = new Vec2D();

    private double x;
    private double y;

    /**
     * Initialize a vector to [0,0]
     */
    public Vec2D() {
        this(0, 0);
    }

    /**
     * Initialize a vector to [x,y]
     * 
     * @param x x extent of the vector
     * @param y y extend of the vector
     */
    public Vec2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * @return the magnitude of the vector
     */
    public double getLength() {
        return (Math.sqrt(getSquaredLength()));
    }

    /**
     * Return the length of the vector before an expensive sqrt operation is
     * carried out. This can be used to compare vector magnitudes relative to
     * each other
     * 
     * @return the length of the vector squared
     */
    public double getSquaredLength() {
        return (x * x) + (y * y);
    }

    /**
     * Normalize the current vector.
     */
    public void normalize() {
        double len = getLength();
        this.x /= len;
        this.y /= len;
    }

    /**
     * @return a normalized copy of this vector
     */
    public Vec2D getNormalized() {
        double length = this.getLength();
        length = (length == 0 ? 1 : length);
        return new Vec2D(x / length, y / length);
    }

    /**
     * Return the dot product of v1 and v2
     * 
     * @param v1 the first vector
     * @param v2 the second vector
     * @return the dot product of v1 and v2
     */
    public static double dotProduct(Vec2D v1, Vec2D v2) {
        return (v1.x * v2.x) + (v1.y * v2.y);
    }

    /**
     * Return the dot product of this vector and the vector v
     * 
     * @param v the other vector
     * @return the dot product of this vector and v
     */
    public double dotProduct(Vec2D v) {
        return (this.x * v.x) + (this.y * v.y);
    }

    /**
     * @return the x
     */
    public double getX() {
        return x;
    }

    /**
     * @param x the x to set
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * @return the y
     */
    public double getY() {
        return y;
    }

    /**
     * @param y the y to set
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * Return the left normal of the vector vec.
     * 
     * @param vec the vector whos normal is to be found
     * @return the normal vector of vec
     */
    public Vec2D getNormal() {
        return new Vec2D(-y, x);
    }

    /**
     * Return the scalar projection of the current vector on the ref vector
     * 
     * @param ref the reference vector onto which to project
     * @return the scalar projection of this vector onto ref.
     */
    public double getScalarProjection(Vec2D ref) {
        return dotProduct(this, ref.getNormalized());
    }

    /**
     * Sums v1 and v2 and returns a vector
     * 
     * @param v1 first vector to add
     * @param v2 second vector to add
     * @return the vector sum of v1 and v2
     */
    public static Vec2D add(Vec2D v1, Vec2D v2) {
        return new Vec2D(v1.getX() + v2.getX(), v1.getY() + v2.getY());
    }

    /**
     * Add the vector v to this vector
     * 
     * @param v the vector to add
     */
    public void add(Vec2D v) {
        x += v.getX();
        y += v.getY();
    }

    /**
     * Subtracts v2 from v1 and return a vector.
     * 
     * @param v1
     * @param v2
     * @return the vector solution of v1 - v2
     */
    public static Vec2D sub(Vec2D v1, Vec2D v2) {
        return new Vec2D(v1.x - v2.x, v1.y - v2.y);
    }

    /**
     * Subtract vector v from this vector
     * 
     * @param v the vector to subtract
     */
    public void sub(Vec2D v) {
        x -= v.getX();
        y -= v.getY();
    }

    /**
     * Returns the 2D cross product of this vector and v1. Also known as the
     * perpendicular dot product. <br>
     * The sign of the answer gives insight into the relative configuration of
     * the two vectors.
     * 
     * @param v1 the vector to perpdot against.
     * @return the perpendicular dot product of this vector and the vector v1.
     */
    public double perpDotProduct(Vec2D v1) {
        return (x * v1.y) - (y * v1.x);
    }

    /**
     * Calculate the vector projection of this vector onto the vector v.
     * 
     * @param v the vector to project onto.
     * @return the vector projection of this vector onto the vector v.
     */
    public Vec2D vecProjection(Vec2D v) {
        double mul = this.dotProduct(v) / v.dotProduct(v);
        return v.getScaled(mul);
    }

    /**
     * Is useful for checking if a point is to the left or right of a given
     * vector. left being positive, right being negative.
     * 
     * @param v1 aka 'a'
     * @param v2 aka 'b'
     * @return a X b
     */
    public static double perpDotProduct(Vec2D v1, Vec2D v2) {
        return (v1.x * v2.y) - (v1.y * v2.x);
    }

    /**
     * Find the angle between this vector and ref.
     * 
     * @param ref the reference vector.
     * @return the angle between this vector and ref.
     */
    public double getRelativeAngle(Vec2D ref) {

        double angle = this.dotProduct(ref);
        angle /= (getLength() * ref.getLength());
        angle = Math.acos(angle);

        return angle;
    }

    /**
     * Uses insertion sort to sort a collection of vectors based on their x
     * values.
     * 
     * @param a the array of vectors to sort.
     * @return an array of vectors sorted by their x values.
     */
    public static Vec2D[] sortByX(Vec2D[] a) {

        Vec2D t;
        for (int i = 1; i < a.length; i++) {
            for (int j = i - 1; j >= 0; j--) {
                if (a[j].getX() > a[j + 1].getX()) {
                    t = a[j];
                    a[j] = a[j + 1];
                    a[j + 1] = t;
                }
                else
                    break;
            }
        }

        return a;
    }

    /**
     * Return a scaled version of this vector
     * 
     * @param k scale value
     * @return
     */
    public Vec2D getScaled(double k) {
        return Vec2D.getScaled(this, k);
    }

    /**
     * Scale up the current vector by the constant 'k'. This is usually for
     * normalized vectors which need to be scaled up.
     * 
     * @param k the scaling constant
     */
    public void scaleBy(double k) {
        x *= k;
        y *= k;
    }

    /**
     * Return a scaled copy of the vector v.
     * 
     * @param v the vector to scale.
     * @param k the scale magnitude.
     * @return v * k.
     */
    public static Vec2D getScaled(Vec2D v, double k) {
        return new Vec2D(v.x * k, v.y * k);
    }

    /**
     * Negates this vector.
     */
    public void negate() {
        x = (x != 0.0 || x != -0.0) ? (x * -1.0) : 0.0;
        y = (y != 0.0 || y != -0.0) ? (y * -1.0) : 0.0;
    }

    /**
     * @return a negated copy of this vector.
     */
    public Vec2D getNegated() {
        return Vec2D.getNegated(this);
    }

    /**
     * @param v the vector to negate.
     * @return a negated copy of the vector v.
     */
    public static Vec2D getNegated(Vec2D v) {
        return new Vec2D((v.x != 0.0 || v.x != -0.0) ? v.x * -1.0 : 0.0,
                (v.y != 0.0 || v.y != -0.0) ? v.y * -1.0 : 0.0);
    }

    /**
     * Draws a point where the vector should be.
     * 
     * @param g2d
     */
    public void drawPoint(Graphics2D g2d) {
        g2d.fillOval((int) x - 1, (int) y - 1, 3, 3);
    }

    /**
     * Draw the full vector.
     * 
     * @param g2d
     */
    public void drawVector(Graphics2D g2d) {
        g2d.drawLine(0, 0, (int) x, (int) y);
        g2d.drawString(String.format("[%.2f, %.2f]", x, y), (int) x - 20,
                (int) y - 10);
        g2d.fillOval((int) x - 1, (int) y - 1, 2, 2);
    }

    /**
     * Draws a vector which has been translated from the origin by trans.
     * 
     * @param g2d
     * @param trans the displacement vector from the origin.
     */
    public void drawTranslatedVec(Graphics2D g2d, Vec2D trans) {
        g2d.drawLine((int) trans.x, (int) trans.y, (int) x, (int) y);
    }

    /**
     * @return a copy of this vector.
     */
    public Vec2D getCopy() {
        return new Vec2D(x, y);
    }

    /**
     * @return a string representation of this vector which can be used to
     *         reconstruct it.
     */
    public String repr() {
        return String.format("new Vec2D(%.5f,%.5f)", x, y);
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f)", x, y);
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        Vec2D other = (Vec2D) obj;
        if ((Double.doubleToLongBits(x) != Double.doubleToLongBits(other.x))
                && (x != other.x)) // also handles -0 == +0. Which is true
            return false;
        if ((Double.doubleToLongBits(y) != Double.doubleToLongBits(other.y))
                && (y != other.y))
            return false;
        return true;
    }

}
