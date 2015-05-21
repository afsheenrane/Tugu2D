package phys2d.entities.shapes.polygons;

import phys2d.entities.Material;
import phys2d.entities.Vec2D;

public final class WorldBound extends Rectangle {
	
	public WorldBound(Vec2D center, double length, double height, double angle) {
		super(center, length, height, angle, Material.REFLECTIUM);
		setMass(0);
	}

	public WorldBound(Vec2D center, double length, double height) {
		this(center, length, height, 0);
	}	
	
	@Override
	public Vec2D getVelocity(){
		return new Vec2D();
	}
	
	@Override
	public void translate(Vec2D trans){
		//Do Nothing.
	}
	
	@Override
	public void move(double dt){
		//Do nothing. Because world bounds are immovable.
	}
	
	@Override
	public void setVelocity(Vec2D velocity){
		//Do nothing. World bounds are immovable.
	}	
}
