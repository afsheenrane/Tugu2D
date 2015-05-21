package phys2d.collisionLogic.collisionCheckers;

import phys2d.collisionLogic.tools.LinePolyTools;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.polygons.Polygon;

public final class CollisionCheckerOld {
		
	/**
	 * Return whether p1 and p2 are colliding using the default algorithm
	 * @param p1
	 * @param p2
	 * @return
	 */
	public static boolean isColliding(Polygon p1, Polygon p2){
		Polygon diff = LinePolyTools.polyDifference(p1, p2);
		return LinePolyTools.isPointInPolygon(diff, Vec2D.ORIGIN);
	}
	
	/**
	 * If p1 and p2 are colliding, return the minimum translation to "uncollide" them.
	 * @param p1
	 * @param p2
	 * @return
	 */
	public static Vec2D getCollisonResolution(Polygon p1, Polygon p2){  //TODO youre actually getting the closest distance if there is no collision. Swept checking
		Polygon diff = LinePolyTools.polyDifference(p1, p2);
		return LinePolyTools.pointDispFromPolySide(diff, Vec2D.ORIGIN);
	}
	
}
