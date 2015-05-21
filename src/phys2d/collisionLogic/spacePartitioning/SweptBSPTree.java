package phys2d.collisionLogic.spacePartitioning;

import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Rectangle;

public class SweptBSPTree extends BSPTree {

	private final double dt;
	
	public SweptBSPTree(Rectangle bounds, int splitMode, int level, double dt) {
		super(bounds, splitMode, level);
		this.dt = dt;
	}

	@Override
	protected int getInsertionSide(Shape s){ 
		Vec2D[] sAabb = s.getSweptAABBbounds(dt);
		return getInsertionSideAABB(sAabb);		
	}
}
