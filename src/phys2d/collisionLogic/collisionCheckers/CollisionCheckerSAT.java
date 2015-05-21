package phys2d.collisionLogic.collisionCheckers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

import phys2d.entities.PhysEntity;
import phys2d.entities.Symmetrical;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Circle;
import phys2d.entities.shapes.polygons.Polygon;

public final class CollisionCheckerSAT {

	public static boolean isCollidingSAT(PhysEntity o1, PhysEntity o2){

		if(o1 instanceof Circle && o2 instanceof Circle)
			return checkCircletoCircleSAT((Circle)o1, (Circle)o2);

		else if(o1 instanceof Circle)
			return checkOBBtoCircleSAT((Polygon)o2, (Circle)o1); 

		else if(o2 instanceof Circle)
			return checkOBBtoCircleSAT((Polygon)o1 , (Circle)o2);

		else
			return checkOBBtoOBBSAT((Polygon)o1, (Polygon)o2);
	}

	private static boolean checkOBBtoOBBSAT(Polygon p1, Polygon p2){
		Vec2D[] p1Norms; 
		Vec2D[] p2Norms;

		if(p1 instanceof Symmetrical)
			p1Norms = ((Symmetrical)p1).getUniqueNormals();
		else
			p1Norms = p1.getNormals();

		if(p2 instanceof Symmetrical)
			p2Norms = ((Symmetrical)p1).getUniqueNormals();
		else
			p2Norms = p2.getNormals();

		HashSet<Vec2D> axes = new HashSet<>();

		for (Vec2D normal : p1Norms){
			axes.add(normal.getNormalized());
		}
		for(Vec2D normal : p2Norms){
			axes.add(normal.getNormalized());
		}

		for (Vec2D axis : axes){

			Vec2D[] p1mm = p1.getMinMax(axis);
			Vec2D[] p2mm = p2.getMinMax(axis);

			if (p1mm[1].getScalarProjection(axis) < p2mm[0].getScalarProjection(axis) || 
					p2mm[1].getScalarProjection(axis) < p1mm[0].getScalarProjection(axis))
				return false;
		}
		return true;
	}

	private static boolean checkOBBtoCircleSAT(Polygon p, Circle c){

		//if no vertex intersection, just use SAT to test the rest of the axes
		ArrayList<Vec2D> axes;

		if(p instanceof Symmetrical)
			axes = new ArrayList<Vec2D>(Arrays.asList(((Symmetrical)p).getUniqueNormals()));
		else
			axes = new ArrayList<Vec2D>(Arrays.asList(p.getNormals()));

		axes.add(Vec2D.sub(getClosestVertexToCircle(p, c), c.getPoints()[0]));

		double circProj; //small optimization to prevent 2x projection computations. Also, readability.
		for (Vec2D axis : axes){

			Vec2D[] pMinMax = p.getMinMax(axis);
			circProj = c.getPoints()[0].getScalarProjection(axis);

			if (pMinMax[1].getScalarProjection(axis) < circProj - c.getRadius() ||
					pMinMax[0].getScalarProjection(axis) > circProj + c.getRadius()){
				return false;
			}
		}
		return true;
	}

	private static boolean checkCircletoCircleSAT(Circle c1, Circle c2){

		//squaring the distance for optimization
		double minSeperation = Math.pow(c1.getRadius() + c2.getRadius(), 2);
		double distance = Vec2D.sub(c1.getPoints()[0], c2.getPoints()[0]).getSquaredLength();

		return distance <= minSeperation;
	}

	private static Vec2D getClosestVertexToCircle(Polygon p, Circle c){

		//Find closest vertex of the polygon to the circle
		Vec2D[] vertices = p.getPoints();
		Vec2D closestVertex = null;
		double closestLen = -1;
		double tLen = -1;

		for (Vec2D vertex : vertices){

			if (closestVertex == null){
				closestVertex = vertex;
				closestLen = Vec2D.sub(c.getPoints()[0], closestVertex).getSquaredLength();
			}
			else{tLen = Vec2D.sub(c.getPoints()[0], vertex).getSquaredLength();
			if (closestLen > tLen){
				closestVertex = vertex;
				closestLen = tLen;
			}
			}
		}
		return closestVertex;
	}

}
