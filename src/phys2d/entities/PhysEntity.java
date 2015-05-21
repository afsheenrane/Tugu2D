package phys2d.entities;


public abstract class PhysEntity {

	protected double mass;
	protected Vec2D velocity;
	
	protected double invMass;
	
	protected PhysEntity(double mass){		
		this.mass = mass;
		this.velocity = new Vec2D();	
	}

	/**
	 * Set the velocity in px/s 
	 * @param velocity the velocity to set
	 */
	public void setVelocity(Vec2D velocity){
		this.velocity = velocity.getCopy();
	}
	
	public double getSpeed(){
		return Math.sqrt(Math.pow(velocity.getX(), 2) + Math.pow(velocity.getY(),2));
	}
	
	/**
	 * Set mass in kg.
	 * @param mass
	 */
	public void setMass(double mass){
		this.mass = mass;
		this.invMass = (mass > 0) ? (1.0 / mass) : 0;
	}
	
	/**
	 * Get the mass in kg
	 * @return
	 */
	public double getMass(){
		return mass;
	}
	
	public double getInvMass(){
		return invMass;
	}
	
	/**
	 * Return the velocity in px/update
	 * @return the velocity in pixels/update
	 */
	public Vec2D getVelocity(){
		return velocity;
	}
	
}

