package phys2d.entities.shapes;

import java.awt.Graphics2D;
import java.util.ArrayList;

import phys2d.entities.Material;
import phys2d.entities.PhysEntity;
import phys2d.entities.Vec2D;

public abstract class Shape extends PhysEntity {

	protected Vec2D[] points;
	protected ArrayList<Vec2D> prevPos;  
	
	protected double angle;  //Angle CCW from +ve axis
	protected double area = 0;
	protected Vec2D centerOfMass;
	protected Material material;
	
	protected Vec2D netForce;
	protected Vec2D lastAccel;
	
	public Shape(Vec2D[] points, Vec2D centerOfMass, double angle, double mass) {
		super(mass);
		this.points = points;
		this.centerOfMass = centerOfMass;
		this.angle = angle;
		this.netForce = new Vec2D(0,0);
		
	}

	public void setAngle(double angle){
		this.angle = angle;
	}
	
	public double getAngle(){
		return angle;
	}
	
	/**
	 * Return the vertices of the polygon
	 * @return vertices of the polygon
	 */
	public Vec2D[] getPoints(){
		return points;
	}
	
	/**
	 * Return the current center of mass of this polygon.
	 * @return the center of mass of this Shape
	 */
	public Vec2D getCOM(){
		return centerOfMass;
	}
	
	/**
	 * @param area the area to set
	 */
	public void setArea(double area) {
		this.area = area;
	}

	/**
	 * @return the area
	 */
	public double getArea(){
		return area;
	}
	
	/**
	 * @param material the material to set
	 */
	public void setMaterial(Material material){
		this.material = material;
		setMass(area * material.getDensity());
	}
	
	/**
	 * @return the material
	 */
	public Material getMaterial() {
		return material;
	}
	
	@Override
	public String toString(){
		String st = this.getClass().getSimpleName() + ": ";
		for (Vec2D p : points){
			st += p + " ";
		}
		st += "COM: " + centerOfMass;
		return st;
	}
		
	public void addForce(Vec2D force) {
		this.netForce.add(force);
		//System.out.println("FORCE ADDED");
		//System.out.println(this.netForce);
	}
	
	public abstract Vec2D[] getMinMax(Vec2D ref);
	
	public abstract Vec2D getMin(Vec2D ref);
	public abstract Vec2D getMax(Vec2D ref);
	
	public abstract void translate(Vec2D translation);
	
	public abstract void move(double dt);
		
	public abstract String repr();
	
	/**
	 * Generate the bounding AABB for this polygon in min-max format
	 * @return the min and max vertices of the bounding AABB for this polygon
	 */
	public abstract Vec2D[] getAABBbounds();
	public abstract Vec2D[] getSweptAABBbounds(double dt);
	
	/**
	 * Generate AABB bound to enclose all the points in pts
	 * @param pts the points to enclose in an AABB
	 * @return the vertices of the AABB in clockwise order
	 */
	public Vec2D[] getFullAABBPts(){
		Vec2D[] AABBmm = getAABBbounds();
		
		return new Vec2D[]{AABBmm[0], new Vec2D(AABBmm[1].getX(), AABBmm[0].getY()),
				AABBmm[1], new Vec2D(AABBmm[0].getX(), AABBmm[1].getY())};		
	}
	
	/**
	 * Draw's the current shape
	 * @param g2d graphics object
	 * @param delta physics delta time
	 */
	public abstract void draw(Graphics2D g2d, double delta);

	/**
	 * @return the netForce
	 */
	public Vec2D getNetForce() {
		return netForce;
	}

}
