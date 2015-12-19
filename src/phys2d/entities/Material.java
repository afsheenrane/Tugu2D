package phys2d.entities;

import java.awt.Color;

public enum Material {

    // COR, DENSITY
    /**
     * COR: 0.828 <br>
     * DENSITY: 1.1 <br>
     * &mu;<sub>S</sub>: 0.95 <br>
     * &mu;<sub>k</sub>: 0.8
     */
    RUBBER(0.828, 1.1, 0.95, 0.8, Color.LIGHT_GRAY),

    /**
     * COR: 0.597 <br>
     * DENSITY: 7.82 <br>
     * &mu;<sub>S</sub>: 0.55 <br>
     * &mu;<sub>k</sub>: 0.3
     */
    STEEL(0.597, 7.82, 0.55, 0.3, Color.GRAY),

    /**
     * COR: 0.05 <br>
     * DENSITY: 0.87 <br>
     * &mu;<sub>S</sub>: 0.3 <br>
     * &mu;<sub>k</sub>: 0.15
     */
    BUTTER(0.05, 0.87, 0.3, 0.15, Color.YELLOW),

    /**
     * COR: 0.25 <br>
     * DENSITY: 2.1 <br>
     * &mu;<sub>S</sub>: 0.85 <br>
     * &mu;<sub>k</sub>: 0.65
     */
    DIRT(0.25, 2.1, 0.85, 0.65, new Color(95, 50, 0)),

    /**
     * COR: 0.18 <br>
     * DENSITY: 2.1 <br>
     * &mu;<sub>S</sub>: 0.55 <br>
     * &mu;<sub>k</sub>: 0.45
     */
    GRASS(0.18, 2.1, 0.55, 0.45, new Color(0, 128, 0)),

    /**
     * COR: 0.13 <br>
     * DENSITY: 1.1 <br>
     * &mu;<sub>S</sub>: 0.75 <br>
     * &mu;<sub>k</sub>: 0.6
     */
    FLESH(0.13, 1.1, 0.75, 0.6, new Color(245, 188, 189)),

    /**
     * COR: 0.6 <br>
     * DENSITY: 0.8 <br>
     * &mu;<sub>S</sub>: 0.45 <br>
     * &mu;<sub>k</sub>: 0.3
     */
    WOOD(0.6, 0.8, 0.45, 0.3, new Color(165, 85, 0, 255)),

    /**
     * COR: 0.66 <br>
     * DENSITY: 2.5 <br>
     * &mu;<sub>S</sub>: 0.4 <br>
     * &mu;<sub>k</sub>: 0.2
     */
    GLASS(0.66, 2.5, 0.4, 0.2, new Color(225, 225, 225, 20)),

    /**
     * COR: 0 <br>
     * DENSITY: 100 <br>
     * &mu;<sub>S</sub>: 2 <br>
     * &mu;<sub>k</sub>: 2
     */
    INERTIUM(0, 100, 2, 2, new Color(0, 0, 0)),

    /**
     * COR: 0.1 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0.9 <br>
     * &mu;<sub>k</sub>: 0.85
     */
    REF10(0.1, 1, 0.9, 0.85, new Color(0, 30, 10)),

    /**
     * COR: 0.2 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0.8 <br>
     * &mu;<sub>k</sub>: 0.75
     */
    REF20(0.2, 1, 0.8, 0.75, new Color(0, 50, 15)),

    /**
     * COR: 0.3 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0.7 <br>
     * &mu;<sub>k</sub>: 0.65
     */
    REF30(0.3, 1, 0.7, 0.65, new Color(0, 70, 20)),

    /**
     * COR: 0.4 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0.6 <br>
     * &mu;<sub>k</sub>: 0.55
     */
    REF40(0.4, 1, 0.6, 0.55, new Color(0, 90, 25)),

    /**
     * COR: 0.5 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0.5 <br>
     * &mu;<sub>k</sub>: 0.45
     */
    REF50(0.5, 1, 0.5, 0.45, new Color(0, 110, 30)),

    /**
     * COR: 0.6 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0.4 <br>
     * &mu;<sub>k</sub>: 0.35
     */
    REF60(0.6, 1, 0.4, 0.35, new Color(0, 130, 35)),

    /**
     * COR: 0.7 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0.3 <br>
     * &mu;<sub>k</sub>: 0.25
     */
    REF70(0.7, 1, 0.3, 0.25, new Color(0, 150, 40)),

    /**
     * COR: 0.8 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0.2 <br>
     * &mu;<sub>k</sub>: 0.15
     */
    REF80(0.8, 1, 0.2, 0.15, new Color(0, 170, 45)),

    /**
     * COR: 0.9 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0.1 <br>
     * &mu;<sub>k</sub>: 0.05
     */
    REF90(0.9, 1, 0.1, 0.05, new Color(0, 190, 50)),

    /**
     * COR: 1 <br>
     * DENSITY: 1 <br>
     * &mu;<sub>S</sub>: 0 <br>
     * &mu;<sub>k</sub>: 0
     */
    REFLECTIUM(1, 1, 0, 0, Color.GREEN);

    protected final double restitution;
    protected final double density; // 1 kg per 1m^2

    protected double staticFric;
    protected double dynFric;

    protected final Color color;

    private Material(double restitution, double density, double statFric,
            double dynFric, Color color) {
        this.restitution = restitution;
        this.density = density;
        this.staticFric = statFric;
        this.dynFric = dynFric;
        this.color = color;
    }

    /**
     * @return the restitution
     */
    public double getRestitution() {
        return restitution;
    }

    /**
     * @return the staticFric
     */
    public double getStaticFric() {
        return staticFric;
    }

    /**
     * @return the dynFric
     */
    public double getDynFric() {
        return dynFric;
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
