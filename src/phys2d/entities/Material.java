package phys2d.entities;

import java.awt.Color;

public enum Material {

	//COR, DENSITY
	/**
	 * COR: 0.828 <br>
	 * DENSITY: 1.1
	 */
	RUBBER(0.828, 1.1, Color.LIGHT_GRAY),
	
	/**
	 * COR: 1 <br>
	 * DENSITY: 1
	 */
	REFLECTIUM(1, 1, Color.GREEN),
	
	/**
	 * COR: 0.597 <br>
	 * DENSITY: 7.82
	 */
	STEEL(0.597, 7.82, Color.GRAY),
	
	/**
	 * COR: 0.05 <br>
	 * DENSITY: 0.87
	 */
	BUTTER(0.05, 0.87, Color.YELLOW),
	
	/**
	 * COR: 0.25 <br>
	 * DENSITY: 2.1
	 */
	DIRT(0.25, 2.1,  new Color(95, 50, 0)),
	
	/**
	 * COR: 0.18 <br>
	 * DENSITY: 2.1
	 */
	GRASS(0.18, 2.1, new Color(0,128,0)),
	
	/**
	 * COR: 0.13 <br>
	 * DENSITY: 1.1
	 */
	FLESH(0.13, 1.1, new Color(245, 188, 189)),
	
	/**
	 * COR: 0.6 <br>
	 * DENSITY: 0.8
	 */
	WOOD(0.6, 0.8, new Color(165, 85, 0, 255)),
	
	/**
	 * COR: 0.66 <br>
	 * DENSITY: 2.5
	 */
	GLASS(0.66, 2.5, new Color(225,225,225,20)),
	
	/**
	 * COR: 0 <br>
	 * DENSITY: 100
	 */
	INERTIUM(0, 100, Color.DARK_GRAY);
	
	protected final double restitution;
	protected final double density; // 1 kg per 1m^2
	
	protected final Color color;
	
	private Material(double restitution, double density, Color color) {
		this.restitution = restitution;
		this.density = density;
		this.color = color;
	}
	
	/**
	 * @return the restitution
	 */
	public double getRestitution() {
		return restitution;
	}
	/**
	 * @return the density
	 */
	public double getDensity() {
		return density;
	}
	
	/**
	 * @return the color
	 */
	public Color getColor(){
		return color;
	}
	
	
	
}
