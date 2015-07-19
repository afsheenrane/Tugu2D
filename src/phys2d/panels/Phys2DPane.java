package phys2d.panels;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.util.ArrayList;

import phys2d.Phys2DMain;
import phys2d.collisionLogic.collisionCheckers.CollisionCheckerGJKEPA2;
import phys2d.collisionLogic.collisionCheckers.SimplexDirStruct;
import phys2d.collisionLogic.collisionManagers.SpeculativeManager2;
import phys2d.collisionLogic.tools.MiscTools;
import phys2d.entities.Material;
import phys2d.entities.Vec2D;
import phys2d.entities.shapes.Circle;
import phys2d.entities.shapes.Shape;
import phys2d.entities.shapes.polygons.Square;
import phys2d.entities.shapes.polygons.WorldBound;

/**
 * @author Afsheen TODO -impulse resolution -swept detection Contact points.
 */

@SuppressWarnings("serial")
public class Phys2DPane extends AnimatedPane {

    private final ArrayList<Shape> entities = new ArrayList<Shape>();

    private final SpeculativeManager2 sm = new SpeculativeManager2(dt);

    public Phys2DPane(int updateRate, int maxFps, int maxFramesSkippable) {
        super(updateRate, maxFps, maxFramesSkippable);
        this.setBackground(Color.black);
    }

    @Override
    public void init() {
        // Add in the world boundaries

        addWorldBounds();
        // populateWithSmallSquares(entities);
        // populateWithSmallCircles(entities);

        Shape s;
        /*
         * int i = 1; for(Material m : Material.values()){ s = new Square(new
         * Vec2D(75 * i++, 600), 25, 0); s.setMaterial(m); entities.add(s); }
         */

        s = new Square(new Vec2D(300, 402), 100, 0);
        // s.setVelocity(new Vec2D(300, 0));
        s.setMaterial(Material.REFLECTIUM);
        entities.add(s);

        s = new Square(new Vec2D(700, 400), 100, 0);
        s.setMaterial(Material.REFLECTIUM);
        // s.setVelocity(new Vec2D(300, 200));
        // entities.add(s);

        s = new Circle(new Vec2D(399, 400), 50);
        s.setMaterial(Material.REFLECTIUM);
        s.setVelocity(new Vec2D(300, 100));
        // entities.add(s);

        s = new Circle(new Vec2D(700, 400), 50);
        s.setMaterial(Material.REFLECTIUM);
        // entities.add(s);

        // tester();

        // System.out.println(LinePolyTools.polyDifference(
        // (Polygon) entities.get(0), (Polygon) entities.get(1)));
        SimplexDirStruct g = CollisionCheckerGJKEPA2
                .getCollisionResolution(entities.get(0), entities.get(1));

        System.out.println(g);

        // System.exit(0);
    }

    @Override
    public void update() {
        sm.runManager(entities);
        // System.out.println(entities.get(4).getVelocity());
        // System.out.println(entities.get(5).getVelocity());
        // System.out.println();
    }

    @Override
    public void render(Graphics2D g2d) {
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        for (Shape entity : entities) {
            entity.draw(g2d, alpha);
        }

    }

    /**
     * Add in the world bounds of the space
     */
    private void addWorldBounds() {
        entities.add(new WorldBound(new Vec2D(0, Phys2DMain.YRES / 2), 20,
                Phys2DMain.YRES + 40)); // left
        entities.add(new WorldBound(new Vec2D(Phys2DMain.XRES / 2, 0),
                Phys2DMain.XRES - 20, 20)); // top
        entities.add(new WorldBound(
                new Vec2D(Phys2DMain.XRES - 5, Phys2DMain.YRES / 2), 20,
                Phys2DMain.YRES + 40)); // right
        entities.add(new WorldBound(
                new Vec2D(Phys2DMain.XRES / 2, Phys2DMain.YRES - 35),
                Phys2DMain.XRES - 20, 20)); // bottom
    }

    private void populateWithSmallSquares(ArrayList<Shape> entities) {
        Shape s;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {

                Vec2D pos = MiscTools.genRandVecs(1, new Vec2D(20, 20),
                        new Vec2D(970, 970))[0];
                double size = 50;
                double ang = 0;// MiscTools.genRandVecs(1, new Vec2D(-(Math.PI *
                               // 2f), 5),new Vec2D(Math.PI * 2f,10))[0].getX();

                s = new Square(pos, size, ang);
                s.setMaterial(Material.REFLECTIUM);
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
                s.setMaterial(Material.REFLECTIUM);
                entities.add(s);
            }
        }
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
