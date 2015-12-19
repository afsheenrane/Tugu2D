package phys2d.panels;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.util.ArrayList;

import phys2d.Phys2DMain;
import phys2d.collisionLogic.collisionCheckers.CollisionCheckerGJKEPA2;
import phys2d.collisionLogic.collisionManagers.DiscreteManager;
import phys2d.collisionLogic.collisionManagers.SpeculativeManager2;
import phys2d.collisionLogic.tools.MiscTools;
import phys2d.entities.Material;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Circle;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Rectangle;
import phys2d.entities.shapes.polygons.Square;
import phys2d.entities.shapes.polygons.WorldBound;

/**
 * @author Afsheen TODO Contact points.
 */

@SuppressWarnings("serial")
public class Phys2DPane extends AnimatedPane {

    private final ArrayList<Shape> entities = new ArrayList<Shape>();

    private final SpeculativeManager2 sm = new SpeculativeManager2(dt);
    private final DiscreteManager dm = new DiscreteManager(dt);

    public Phys2DPane(int updateRate, int maxFps, int maxFramesSkippable) {
        super(updateRate, maxFps, maxFramesSkippable);
        this.setBackground(Color.black);
    }

    @Override
    public void init() {
        // Add in the world boundaries

        addWorldBounds("a");
        // populateWithSmallSquares(entities);
        // populateWithSmallCircles(entities);

        addRefSquares();

        Shape s;

        s = new Rectangle(new Vec2D(700, 700), 5, 300);
        s.setMaterial(Material.RUBBER);
        entities.add(s);

        s = new Square(new Vec2D(150, 875), 1, 0);
        s.setMaterial(Material.REF90);
        s.setVelocity(new Vec2D(7000, 0));
        //entities.add(s);

        s = new Square(new Vec2D(630, 805), 1, 0);
        s.setMaterial(Material.REF70);
        s.setVelocity(new Vec2D(7500, 0));
        //entities.add(s);

        // tester();

        // System.out.println(
        // LinePolyTools.polyDifference(entities.get(0), entities.get(1)));
        // System.exit(0);
    }

    int upCt = 0;

    @Override
    public void update() {
        sm.runManager(entities);
        //System.out.println(++upCt + ": " + entities.get(1).getCOM() + " " + entities.get(1).getVelocity());
    }

    @Override
    public void render(Graphics2D g2d) {
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        for (Shape entity : entities) {
            entity.draw(g2d, alpha);
        }

        sm.getCollisionTree().draw(g2d);

    }

    /**
     * Add in the specified world boundaries.
     * 
     * @param s the boundaries to add in. l for left, t for top, r for right, b
     *            for bottom, or a for all.
     */
    private void addWorldBounds(String s) {
        // Left
        if (s.indexOf('a') != -1 || s.indexOf('l') != -1) {
            entities.add(new WorldBound(new Vec2D(Phys2DMain.XRES * -1.5, -20), new Vec2D(10, Phys2DMain.YRES + 20)));
        }

        // Bottom
        if (s.indexOf('a') != -1 || s.indexOf('b') != -1) {
            entities.add(new WorldBound(new Vec2D(10, Phys2DMain.YRES * -1.5), new Vec2D(Phys2DMain.XRES - 10, 40)));
        }

        // Right
        if (s.indexOf('a') != -1 || s.indexOf('r') != -1) {
            entities.add(new WorldBound(new Vec2D(Phys2DMain.XRES - 15, -20),
                    new Vec2D(Phys2DMain.XRES * 1.5, Phys2DMain.YRES + 20)));
        }
        // Top
        if (s.indexOf('a') != -1 || s.indexOf('t') != -1) {
            entities.add(new WorldBound(new Vec2D(-10, Phys2DMain.YRES - 10),
                    new Vec2D(Phys2DMain.XRES + 10, Phys2DMain.YRES * 1.5)));
        }
        for (Shape w : entities) {
            System.out.println(w);
        }

    }

    private void populateWithSmallSquares(ArrayList<Shape> entities) {
        Shape s;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {

                Vec2D pos = MiscTools.genRandVecs(1, new Vec2D(20, 20),
                        new Vec2D(970, 970))[0];
                double size = 50;
                double ang = 0;// MiscTools.genRandVecs(1, new Vec2D(-(Math.PI *
                               // 2f), 5),new Vec2D(Math.PI * 2f,10))[0].getX();

                s = new Square(pos, size, ang);
                // s.setMaterial(Material.REFLECTIUM);
                s.setVelocity(MiscTools.genRandVecs(1, new Vec2D(-150, -150),
                        new Vec2D(150, 150))[0]);
                // s.setMass(5);
                // s.setVelocity(new Vec2D(40,70), updateRate);
                entities.add(s);
            }
        }
    }

    private void populateWithSmallCircles(ArrayList<Shape> entities) {
        Shape s;
        int radius = 20;
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 2; j++) {
                s = new Circle(
                        MiscTools.genRandVecs(1,
                                new Vec2D(50 + radius, 50 + radius),
                                new Vec2D(900 - radius, 900 - radius))[0],
                        radius);
                s.setVelocity(MiscTools.genRandVecs(1, new Vec2D(-200, -200),
                        new Vec2D(200, 200))[0]);
                // s.setMaterial(Material.REFLECTIUM);
                entities.add(s);
            }
        }
    }

    private void addRefSquares() {
        Shape s;
        s = new Square(new Vec2D(50, 800), 40, 0);
        s.setMaterial(Material.INERTIUM);
        entities.add(s);
        s = new Square(new Vec2D(100, 800), 40, 0);
        s.setMaterial(Material.REF10);
        entities.add(s);
        s = new Square(new Vec2D(150, 800), 40, 0);
        s.setMaterial(Material.REF20);
        entities.add(s);
        s = new Square(new Vec2D(200, 800), 40, 0);
        s.setMaterial(Material.REF30);
        entities.add(s);
        s = new Square(new Vec2D(250, 800), 40, 0);
        s.setMaterial(Material.REF40);
        entities.add(s);
        s = new Square(new Vec2D(300, 800), 40, 0);
        s.setMaterial(Material.REF50);
        entities.add(s);
        s = new Square(new Vec2D(350, 800), 40, 0);
        s.setMaterial(Material.REF60);
        entities.add(s);
        s = new Square(new Vec2D(400, 800), 40, 0);
        s.setMaterial(Material.REF70);
        entities.add(s);
        s = new Square(new Vec2D(450, 800), 40, 0);
        s.setMaterial(Material.REF80);
        entities.add(s);
        s = new Square(new Vec2D(500, 800), 40, 0);
        s.setMaterial(Material.REF90);
        entities.add(s);
        s = new Square(new Vec2D(550, 800), 40, 0);
        s.setMaterial(Material.REFLECTIUM);
        entities.add(s);
    }

    private void tester() {
        double dt = 0;
        int tests = 3000;
        int frames = 5000;
        for (int j = 0; j < tests; j++) {
            long t = System.nanoTime();
            for (int i = 0; i < frames; i++) {
                // CollisionCheckerMPR.isColliding(entities.get(0),
                // entities.get(1));
                CollisionCheckerGJKEPA2.getCollisionResolution(entities.get(0),
                        entities.get(1));
            }

            dt += (System.nanoTime() - t) / 1e6;
        }
        System.out.println("DEF: " + dt / tests);
        System.exit(0);
    }

    private Color randCol() {
        float hue = (float) Math.random();
        int rgb = Color.HSBtoRGB(hue, 0.5f, 0.5f);
        return new Color(rgb);
    }

} // END CLASS
