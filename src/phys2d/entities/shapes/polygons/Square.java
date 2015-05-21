package phys2d.entities.shapes.polygons;

import phys2d.entities.Material;
import phys2d.entities.Vec2D;

public class Square extends Rectangle{

	/**
	 * Initialize a square with the center pos and length. With a velocity of [0,0]
	 * @param pos center of the square
	 * @param length side lengths of the square
	 * @param angle the angle of rotation
	 */
	public Square(Vec2D pos, double length, double angle){
		super(pos, length, length, angle, Material.RUBBER);
	}
}
