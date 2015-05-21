package phys2d.collisionLogic.tools;

import java.util.ArrayList;
import java.util.Arrays;

import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Circle;
import phys2d.entities.shapes.polygons.Polygon;

/**
 * A collection of methods regarding lines and polygons. Including point tests,
 * minkowski sum/differences, and finding the displacement of points from lines.
 * @author afsheen
 *
 */

public final class LinePolyTools {

	private static final Vec2D ORIGIN = new Vec2D();
	
	/**
	 * Return the displacement to the closest side of the polygon when p lies inside the polygon.
	 * Else return the zero vector
	 * @param poly the polygon to check containment within
	 * @param p to point to check if it is contained in poly
	 * @return the exact displacement vector the point is from the closest side of the polygon.
	 * zero vector if the point is not inside the polygon.
	 */
	public static Vec2D pointDispFromPolySide(Polygon poly, Vec2D p){
		Vec2D[] result = pointInPolyTester(poly, p, 1);
		return (result.length > 0) ? result[0] : ORIGIN;
	}
	
	/**
	 * Returns whether the point p is inside the polygon poly
	 * @param poly The polygon to check if p is contained within
	 * @param p The point to check
	 * @return if the point is inside the polygon
	 */
	public static boolean isPointInPolygon(Polygon poly, Vec2D p){
		return (pointInPolyTester(poly, p, 0).length > 0) ? true : false;
	}
	
	/**
	 * mode 0 = just yes/no 1 = exact displacement vector from closest side
	 * @param poly
	 * @param p
	 * @param mode
	 * @return if mode is 0, return an empty vec2d array if the points is not in poly. Otherwise a populated vec2D[]
	 * populated = true
	 * empty = false
	 * If mode is 1, return the displacement vector of the point to the closest polygon edge
	 */
	private static Vec2D[] pointInPolyTester(Polygon poly, Vec2D p, int mode){
		Vec2D[] polyPts = poly.getPoints();
		
		final Vec2D[] TRUE =  new Vec2D[]{ORIGIN};
		final Vec2D[] FALSE =  new Vec2D[]{};
		
		if(mode == 0 && polyPts.length == 3)
			return (isPointInTriangle(poly, p).size() == 3) ? TRUE : FALSE;
		
		Polygon triangle; 
		ArrayList<Vec2D> subTriPts = new ArrayList<Vec2D>(3);
		
		//Check which voronoi region of the polygon p is in.
		for(int i = 0; i < polyPts.length - 1; i++){
			triangle = new Polygon(new Vec2D[]{polyPts[i], poly.getCOM(), polyPts[i + 1]}, 0);
			
			subTriPts = isPointInTriangle(triangle, p);
			
			if(subTriPts.size() == 3){  //if a valid voronoi region is found
				if(mode == 0)  //if boolean answer request, just return "true"
					return TRUE;
				else if(mode == 1){  //otherwise compute the length from the polygon side
					subTriPts.remove(poly.getCOM());  //remaining two vectors form line
					return new Vec2D[]{ptToLineDisp(p, new Vec2D[]{subTriPts.get(0), subTriPts.get(1)})};
				}
				else
					System.out.println("FLOATING POINT ERROR IN PHYSTOOLKIT. FIX ME");
			}
		}	
		//for the final pts. Where the last point in polygon wraps to the first
		triangle = new Polygon(new Vec2D[]{polyPts[polyPts.length - 1], poly.getCOM(), polyPts[0]}, 0);
		
		if(isPointInTriangle(triangle, p).size() == 3){  //if the point is in the final region
			if(mode == 0)  //bool requested
				return TRUE;
			else  //exact dist requested
				return new Vec2D[]{ptToLineDisp(p, new Vec2D[]{polyPts[0], polyPts[polyPts.length - 1]})};
		}
		else  //if it is not in the final region return that the point is not in the polygon
			return FALSE;		
	}
	
	/**
	 * Return whether the point is in the triangle or not
	 * @param triangle the triangle to check 
	 * @param p the point to check
	 * @return an empty arraylist if the point is not contained, or an arraylist of size 3
	 * if it is.
	 */
	private static ArrayList<Vec2D> isPointInTriangle(Polygon triangle, Vec2D p){
		Vec2D[] triPts = triangle.getPoints();
		
		//Optimized to use short-circuit evaluation
		if(getLeftOrRight(triPts[0], triPts[1], p) > 0 || 
				getLeftOrRight(triPts[1], triPts[2], p) > 0 ||
				getLeftOrRight(triPts[2], triPts[0], p) > 0) {
			return new ArrayList<Vec2D>(0);
		}
		
		return new ArrayList<Vec2D>(Arrays.asList(triPts)); //return the triangle points if it is contained
	}

	
	/**
	 * Return the displacement vector of a a point 'p' to line 'line'
	 * @param p
	 * @param line
	 * @return
	 */
	public static Vec2D ptToLineDisp(Vec2D p, Vec2D[] line){
		
		//Assuming cw order of points has been conserved throughout calculations.
		Vec2D lineNorm = Vec2D.sub(line[1], line[0]).getNormal();	
		Vec2D relVec = Vec2D.sub(p, line[0]);
		double distFromEdge = Math.abs(relVec.getScalarProjection(lineNorm));
		
		lineNorm.normalize();
		lineNorm.scaleBy(distFromEdge);
		
		return lineNorm;
	}
	
	/**
	 * Return the displacement vector of a point 'p' from the line segment lineSeg.
	 * ie. keeping the lineSeg as our reference frame, what is the displacement of p
	 * from it?
	 * @param p the point to check
	 * @param lineSeg the end points of the line segment
	 * @return the displacement of p from lineSeg
	 */
	public static Vec2D ptToLineSegDisp(Vec2D p, Vec2D[] lineSeg){
		
		if(lineSeg.length == 1)
			return Vec2D.sub(lineSeg[0], p);
		
		//First translate p and lineSeg to the ORIGIN. That is, make lineSeg into
		//a vector and translate p so that the displacement from linSeg is the same
		Vec2D transP = Vec2D.sub(p, lineSeg[0]);
		Vec2D transLineSeg = Vec2D.sub(lineSeg[lineSeg.length - 1], lineSeg[0]);		
		//doing lineSeg.length - 1, because this method is mostly going to be
		//called by the GJK collision checker. It is possible that the lineSeg is only
		//a single point. Hence, this is being used to prevent an index out of bounds exception.
				
		//Scalar project p onto lineSeg. If negative, then disp is the translated p
		double proj = transP.getScalarProjection(transLineSeg);
		
		if(proj <= 0)
			return transP;
		
		//if positive and past endpoint, then do trans-p - endpoint
		else if(proj * proj > transLineSeg.getSquaredLength())
			return Vec2D.sub(transP, transLineSeg);
		
		else
			return ptToLineDisp(p, lineSeg);
	}
	
	/**
	 * Return the point on the lineSeg which is closest to the origin. 
	 * Not restricted to vertices. It is the true closest point anywhere on the line.
	 * @param lineSeg the line segment to get the closest point from
	 * @return
	 */
	public static Vec2D getClosestPtToOrigin(Vec2D[] lineSeg){
		if(lineSeg.length == 1)
			return lineSeg[0];

		
		Vec2D lineSegVec = Vec2D.sub(lineSeg[lineSeg.length - 1], lineSeg[0]);
		Vec2D relOrigin = Vec2D.getNegated(lineSeg[0]);
		
		double proj = relOrigin.getScalarProjection(lineSegVec);
		
		if(proj <= 0)  //if its before the 0 point
			return lineSeg[0];
		
		else if(proj * proj >= lineSegVec.getSquaredLength())  //if its past the 1 point
			return lineSeg[lineSeg.length - 1];
		
		else
			return Vec2D.add(Vec2D.getScaled(lineSegVec.getNormalized(), proj), lineSeg[0]);
		
		
	}
	
	/**
	 * Same as above, accepting an arraylist instead of an array
	 * @param lineSeg
	 * @return
	 */
	public static Vec2D getClosestPtToOrigin(ArrayList<Vec2D> lineSeg){
		return getClosestPtToOrigin(lineSeg.toArray(new Vec2D[]{}));
	}
	
	/**
	 * Returns whether the point p is to the left or right of the line v1v2.
	 * Left = center = 1
	 * right = -1
	 * @param v1 start point of line
	 * @param v2 end point of line
	 * @param p the point to check
	 * @return whether p is to the left or right of v1v2
	 */
	public static int getLeftOrRight(Vec2D v1, Vec2D v2, Vec2D p){
		final double TOLERANCE =  0.1;
		double result;
		result = Vec2D.perpDotProduct(Vec2D.sub(v2, v1), Vec2D.sub(p, v1));
		
		if(Math.abs(result) < TOLERANCE) {
			//result = -1;
			return -1;
		}
		
		return (result > 0) ? 1 : -1;  //return 1 if result is positive(left), -1 otherwise (right). was >=
	}

	/**
	 * Generating a convex hull from the entered points
	 * @param points the points to generate the hull from
	 * @return the convex hull polygon
	 */
	public static Polygon generateConvexHull(Vec2D[] points){

		if(points.length <= 3) return new Polygon(points, 0);

		ArrayList<Vec2D> hullPoints = new ArrayList<Vec2D>(points.length / 2);
		ArrayList<Vec2D> remainingPoints = new ArrayList<Vec2D>(Arrays.asList(points));

		ArrayList<Vec2D> leftPts = new ArrayList<Vec2D>();
		ArrayList<Vec2D> rightPts = new ArrayList<Vec2D>();

		//find x-minmax
		Vec2D[] xMM = MiscTools.getMinMax(points, new Vec2D(1,0));

		//add these to the hull
		hullPoints.add(xMM[0]);
		hullPoints.add(xMM[1]);

		//remove them from the remaining pts
		remainingPoints.remove(xMM[0]);
		remainingPoints.remove(xMM[1]);

		//allocate into left and right side points
		for(Vec2D pt : remainingPoints){
			//if(Vec2D.perpDotProduct(Vec2D.sub(xMM[1], xMM[0]), Vec2D.sub(pt, xMM[0])) >= 0)
			if(getLeftOrRight(xMM[0], xMM[1], pt) > 0)
				leftPts.add(pt);
			else
				rightPts.add(pt);
		}

		genHull(xMM[0], xMM[1], leftPts, hullPoints);  //gen hull from left points
		genHull(xMM[1], xMM[0], rightPts, hullPoints);  //gen hull from right points
		return new Polygon(hullPoints.toArray(new Vec2D[]{}));
	}

	/**
	 * Recursive helper of generateConvexHull
	 * @param p1
	 * @param p2
	 * @param pointSet
	 * @param hull
	 */
	private static void genHull (Vec2D p1, Vec2D p2, ArrayList<Vec2D> pointSet, ArrayList<Vec2D> hull){
		int insertionPt = hull.indexOf(p2);  //where the point will be added 
		if(pointSet.size() == 0)
			return;

		//if only 1 point in the set, it must be on the hull
		if(pointSet.size() == 1){
			Vec2D t = pointSet.get(0);
			hull.add(insertionPt, t);
			pointSet.remove(t);
			return;
		}

		//find the point that is furthest away from line p1p2
		Vec2D norm = Vec2D.sub(p2, p1).getNormal();
		Vec2D relVec;
		Vec2D furthestPt = new Vec2D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
		double furthestDist = Double.NEGATIVE_INFINITY;
		double dist;
		for (Vec2D pt : pointSet){
			relVec = Vec2D.sub(pt, p1);
			dist = relVec.dotProduct(norm);
			if(furthestDist < dist){
				furthestPt = pt;
				furthestDist = dist;
			}
		}

		//add furthest point to the hull
		pointSet.remove(furthestPt);
		hull.add(furthestPt);

		//find points to the left of p1-fur
		ArrayList<Vec2D> leftP1Fur = new ArrayList<Vec2D>();
		for(Vec2D pt : pointSet){
			if (getLeftOrRight(p1, furthestPt, pt) > 0)
				leftP1Fur.add(pt);	
		}

		//find points to the left of fur-p2
		ArrayList<Vec2D> leftFurP2 = new ArrayList<Vec2D>();
		for(Vec2D pt : pointSet){
			if(getLeftOrRight(furthestPt, p2, pt) > 0)
				leftFurP2.add(pt);
		}

		genHull(p1, furthestPt, leftP1Fur, hull); 
		genHull(furthestPt, p2, leftFurP2, hull);
	}

	/**
	 * Depending on the mode, produces the minkowski sum or difference of the two polygons
	 * @param p1 first polygon
	 * @param p2 second polygon
	 * @param mode 1 = sum. -1 = difference
	 * @return
	 */
	private static Polygon polySumDif(Polygon p1, Polygon p2, int mode){
		ArrayList<Vec2D> summedPts = new ArrayList<Vec2D>();

		Vec2D[] p1Pts = p1.getPoints();
		Vec2D[] p2Pts = p2.getPoints();

		for(Vec2D p1p : p1Pts){
			for(Vec2D p2p : p2Pts){
				if(mode > 0)
					summedPts.add(Vec2D.add(p1p, p2p));
				else
					summedPts.add(Vec2D.sub(p1p, p2p));
			}
		}

		return generateConvexHull(summedPts.toArray(new Vec2D[]{}));
	}
	
	private static Circle circSumDif(Circle c1, Circle c2, int mode){
		Vec2D center = (mode > 0) ? Vec2D.add(c1.getPoints()[0], c2.getPoints()[0]) : Vec2D.sub(c1.getPoints()[0], c2.getPoints()[0]);
		double radius = c1.getRadius() + c2.getRadius();
		
		return new Circle(center, radius);
	}
	
	/**
	 * Return the minkowski sum of p1 and p2
	 * @param p1 first polygon
	 * @param p2 second polygon
	 * @return the polygon which is the minkowski sum of p1 and p2. (p1 + p2)
	 */
	public static Polygon polySum(Polygon p1, Polygon p2){
		return polySumDif(p1, p2, 1);
	}

	/**
	 * Return the minkowski difference of p1 and p2
	 * @param p1 first polygon
	 * @param p2 second polygon
	 * @return the polygon which is the minkowski difference of p1 and p2. (p1 - p2)
	 */
	public static Polygon polyDifference(Polygon p1, Polygon p2){
		return polySumDif(p1, p2, -1);
	}
	
	/**
	 * Return the minkowski sum of c1 and c2
	 * @param c1 the first circle
	 * @param c2 the second circle
	 * @return the circle which is the minkowski sum of c1 and c2. (c1 + c2)
	 */
	public static Circle circSum(Circle c1, Circle c2){
		return circSumDif(c1, c2, 1);
	}
	
	/**
	 * Return the minkowski difference of c1 and c2
	 * @param c1 the first circle
	 * @param c2 the second circle
	 * @return the circle which is the minkowski difference of c1 and c2. (c1 - c2)
	 */
	public static Circle circDifference(Circle c1, Circle c2){
		return circSumDif(c1, c2, -1);
	}
	
}
