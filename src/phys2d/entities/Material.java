package phys2d.entities;

import java.awt.Color;

public enum Material {

    // COR, DENSITY
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
    DIRT(0.25, 2.1, new Color(95, 50, 0)),

    /**
     * COR: 0.18 <br>
     * DENSITY: 2.1
     */
    GRASS(0.18, 2.1, new Color(0, 128, 0)),

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
    GLASS(0.66, 2.5, new Color(225, 225, 225, 20)),

    /**
     * COR: 0 <br>
     * DENSITY: 100
     */
    INERTIUM(0, 100, new Color(0, 0, 0)),

    /**
     * COR: 0.1 <br>
     * DENSITY: 1
     */
    REF10(0.1, 1, new Color(0, 30, 10)),

    /**
     * COR: 0.2 <br>
     * DENSITY: 1
     */
    REF20(0.2, 1, new Color(0, 50, 15)),

    /**
     * COR: 0.3 <br>
     * DENSITY: 1
     */
    REF30(0.3, 1, new Color(0, 70, 20)),

    /**
     * COR: 0.4 <br>
     * DENSITY: 1
     */
    REF40(0.4, 1, new Color(0, 90, 25)),

    /**
     * COR: 0.5 <br>
     * DENSITY: 1
     */
    REF50(0.5, 1, new Color(0, 110, 30)),

    /**
     * COR: 0.6 <br>
     * DENSITY: 1
     */
    REF60(0.6, 1, new Color(0, 130, 35)),

    /**
     * COR: 0.7 <br>
     * DENSITY: 1
     */
    REF70(0.7, 1, new Color(0, 150, 40)),

    /**
     * COR: 0.8 <br>
     * DENSITY: 1
     */
    REF80(0.8, 1, new Color(0, 170, 45)),

    /**
     * COR: 0.9 <br>
     * DENSITY: 1
     */
    REF90(0.9, 1, new Color(0, 190, 50));

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
    public Color getColor() {
        return color;
    }

}
