package phys2d.collisionLogic.spacePartitioning;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;

import phys2d.Phys2DMain;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Square;
import phys2d.entities.shapes.polygons.WorldBound;

public class CollisionGrid {

	private double tileSize;
	private static final Vec2D OFFSET = new Vec2D(-50, -50);
	private final int numTiles;
	

	public CollisionGrid(Shape[] shapes){
		this.tileSize = Math.max(Phys2DMain.XRES, Phys2DMain.YRES) / 8;
		double[] t = computeTileSize(shapes);
		tileSize = t[0];
		numTiles = (int)t[1];
	}

	private double findAverageShapeSize(Shape[] shapes){
		if(shapes.length > 4){
			Vec2D[] aabb;
			double avgBound = 0;

			for (Shape s : shapes){
				if(!(s instanceof WorldBound)){
					aabb = s.getAABBbounds();
					avgBound += Math.abs(aabb[0].getX() - aabb[aabb.length - 1].getX());  //abs of min - max			
					avgBound += Math.abs(aabb[0].getY() - aabb[aabb.length - 1].getY()); 
					avgBound /= 2;
				}
			}
			avgBound /= (shapes.length - 4); // -4 world bounds
			return avgBound;
		}
		else{
			return tileSize;
		}
	}

	private double findMaxShapeSize(Shape[] shapes){
		Vec2D[] aabb;
		double max = Double.NEGATIVE_INFINITY;

		double t;

		for(Shape s : shapes){
			if(!(s instanceof WorldBound)){
				aabb = s.getAABBbounds();
				t =  Math.abs(aabb[1].getX() - aabb[0].getX());
				max = t > max ? t : max;

				t =  Math.abs(aabb[0].getY() - aabb[3].getY());
				max = t > max ? t : max;
			}
		}
		return max;
	}

	public Square[][] generateGrid(Shape[] shapes){
		Square[][] tiles = new Square[numTiles][numTiles];

		for(int i = 0; i < numTiles; i++){
			for(int j = 0; j < numTiles; j++){
				tiles[i][j] = new Square(new Vec2D((i * tileSize) + (tileSize / 2) + OFFSET.getX(), (j * tileSize) + (tileSize / 2) + OFFSET.getY()), tileSize, 0);
			}
		}
		return tiles;
	}

	/**
	 * Compute the tileSize
	 * @param shapes
	 * @return tileSize, numTiles
	 */
	private double[] computeTileSize(Shape[] shapes){
		int remainder = 0;
		int res = Math.max(Phys2DMain.XRES, Phys2DMain.YRES) - (int)OFFSET.getX();

		tileSize = findAverageShapeSize(shapes);

		//tileSize = findMaxShapeSize(shapes);

		tileSize *= 1.5;  //1.25 for maxShapeSize, 1.5 for avgShapeSize
		
		try{  //TODO
			remainder = res % (int)tileSize;
		} catch(Exception e){
			System.out.println("resolution: " + res);
			System.out.println("tileSize: " + tileSize);
			System.out.println("avgTilesz: " + findAverageShapeSize(shapes));
			e.printStackTrace();
			System.exit(0);
		}
		if(remainder < tileSize / 2){
			tileSize += remainder / ((res - remainder) / tileSize);  //remainder / numTiles (in perfect world)
			return new double[]{tileSize, Math.round(res / tileSize)};
		}
		else{
			return new double[]{tileSize, Math.round(res / tileSize) + 1};
		}
	}

	public Shape[][] getCollisionGroups(ArrayList<Shape> shapes){
		return getCollisionGroups(shapes.toArray(new Shape[]{}));
	}

	/**
	 * Return groups of polygons between which narrow phase collision detection needs to be conducted
	 * @param shapes
	 * @return
	 */
	public Shape[][] getCollisionGroups(Shape[] shapes){

		HashSet<Shape[]> collisionGroups;
		HashMap<Vec2D, HashSet<Shape>> groups = new HashMap<Vec2D, HashSet<Shape>>();
		Vec2D[] aabb;
		Vec2D key;

		for(Shape s : shapes){
			aabb = s.getAABBbounds();  //the aabb extents of the shape to check
			int[] xRng = new int[2];  //x extent in grid 
			int[] yRng = new int[2];  //y extent in grid

			for(int i = 0; i < aabb.length; i++){  //create the x and y extents of the aabb in the grid  
				xRng[i] = (int)(aabb[i].getX() / tileSize);
				yRng[i] = (int)(aabb[i].getY() / tileSize);
			}

			//add those extents into the grid as being occupied by Shape 's' 
			for(int x = xRng[0]; x <= xRng[1]; x++){
				for(int y = yRng[0]; y <= yRng[1]; y++){
					key = new Vec2D(x,y);

					if(!groups.containsKey(key))
						groups.put(key, new HashSet<Shape>());

					groups.get(key).add(s);
				}
			}
		}

		Collection<HashSet<Shape>> values = groups.values();
		collisionGroups =  new HashSet<Shape[]>();

		for(HashSet<Shape> value : values){
			if(value.size() > 1)  //only if there are atleast 2 shapes inside a tile, group them
				collisionGroups.add(value.toArray(new Shape[]{}));  
		}
		return collisionGroups.toArray(new Shape[][]{});
	}

	public double getTileSize(){
		return tileSize;
	}

	public int getNumTiles(){
		return numTiles;
	}

}
