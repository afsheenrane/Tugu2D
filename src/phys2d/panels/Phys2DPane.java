package phys2d.panels;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.util.ArrayList;

import phys2d.Phys2DMain;
import phys2d.collisionLogic.collisionManagers.CollisionManager;
import phys2d.collisionLogic.collisionManagers.SpeculativeManager2;
import phys2d.collisionLogic.tools.LinePolyTools;
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

    private CollisionManager collManager;

    public Phys2DPane(int updateRate, int maxFps, int maxFramesSkippable) {
        super(updateRate, maxFps, maxFramesSkippable);
        this.setBackground(Color.black);
    }

    @Override
    public void init() {
        collManager = new SpeculativeManager2(dt);

        // Add in the world boundaries

        addWorldBounds("a");
        //populateWithSmallSquares(entities, 2, new Random().nextLong());
        populateWithSmallSquares(entities, 200, -111089002341966575l);

        //populateWithSmallCircles(entities, 30, new Random().nextLong());
        populateWithSmallCircles(entities, 200, 3801484226869149488l);
        //addRefSquares(700);

        Shape s;

        s = new Rectangle(new Vec2D(700, 200), 30, 300);
        s.setMaterial(Material.RUBBER);
        //entities.add(s);

        s = new Square(new Vec2D(980, 400), 30, 0); //970
        s.setMaterial(Material.REF60);
        s.setVelocity(new Vec2D(100, 0));
        //entities.add(s);

        s = new Square(new Vec2D(550, 200), 50, 0);
        s.setMaterial(Material.REF60);
        s.setVelocity(new Vec2D(20, -20));
        //entities.add(s);

        s = new Circle(new Vec2D(300, 150), 30); //For ground contact: [300,70],30
        s.setMaterial(Material.REF60);
        s.setVelocity(new Vec2D(-200, 80));
        //entities.add(s);

        collManager.setForceOfGravity(0);

        // tester();

        System.out.println(LinePolyTools.polyDifference(entities.get(0), entities.get(1)));

        // System.exit(0);
    }

    int upCt = 0;
    long total = 0;

    @Override
    public void update() {
        long t = System.nanoTime();
        collManager.runManager(entities);
        total += System.nanoTime() - t;
        //System.out.println((System.nanoTime() - t) / 1e6 + "    " + dt * 1000);
        System.out.println(++upCt);
        if (upCt == 1000) {
            System.out.println((total / 1000) / 1e6);
            //System.out.println("break pt");
        }
    }

    @Override
    public void render(Graphics2D g2d) {
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        for (Shape entity : entities) {
            entity.draw(g2d, alpha);
            //entity.getVelocity().drawTranslatedVec(g2d, entity.getCOM());
        }

        //((SpeculativeManager2) collManager).getCollisionTree().draw(g2d);

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

    private void populateWithSmallSquares(ArrayList<Shape> entities, int num, long seed) {

        Shape s;
        Vec2D[] pos = MiscTools.genRandVecs(num, new Vec2D(20, 20), new Vec2D(970, 970), seed);
        Vec2D[] vel = MiscTools.genRandVecs(num, new Vec2D(-250, -250), new Vec2D(250, 250), seed);
        double size = 20;
        double ang = 0;

        Material m = Material.REF80;

        for (int i = 0; i < num; i++) {
            s = new Square(pos[i], size, ang);
            s.setMaterial(m);
            s.setVelocity(vel[i]);
            entities.add(s);
        }
    }

    private void populateWithSmallCircles(ArrayList<Shape> entities, int num, long seed) {

        Shape s;
        double radius = 10;
        Vec2D[] pos = MiscTools.genRandVecs(num, new Vec2D(20, 20), new Vec2D(970, 970), seed);
        Vec2D[] vel = MiscTools.genRandVecs(num, new Vec2D(-250, -250), new Vec2D(250, 250), seed);
        Material m = Material.REF80;

        for (int i = 0; i < num; i++) {
            s = new Circle(pos[i], radius);
            s.setMaterial(m);
            s.setVelocity(vel[i]);
            entities.add(s);
        }
    }

    private void addRefSquares(double height) {
        Shape s;
        s = new Square(new Vec2D(50, height), 40, 0);
        s.setMaterial(Material.INERTIUM);
        entities.add(s);
        s = new Square(new Vec2D(100, height), 40, 0);
        s.setMaterial(Material.REF10);
        entities.add(s);
        s = new Square(new Vec2D(150, height), 40, 0);
        s.setMaterial(Material.REF20);
        entities.add(s);
        s = new Square(new Vec2D(200, height), 40, 0);
        s.setMaterial(Material.REF30);
        entities.add(s);
        s = new Square(new Vec2D(250, height), 40, 0);
        s.setMaterial(Material.REF40);
        entities.add(s);
        s = new Square(new Vec2D(300, height), 40, 0);
        s.setMaterial(Material.REF50);
        entities.add(s);
        s = new Square(new Vec2D(350, height), 40, 0);
        s.setMaterial(Material.REF60);
        entities.add(s);
        s = new Square(new Vec2D(400, height), 40, 0);
        s.setMaterial(Material.REF70);
        entities.add(s);
        s = new Square(new Vec2D(450, height), 40, 0);
        s.setMaterial(Material.REF80);
        entities.add(s);
        s = new Square(new Vec2D(500, height), 40, 0);
        s.setMaterial(Material.REF90);
        entities.add(s);
        s = new Square(new Vec2D(550, height), 40, 0);
        s.setMaterial(Material.REFLECTIUM);
        entities.add(s);
    }

    private Color randCol() {
        float hue = (float) Math.random();
        int rgb = Color.HSBtoRGB(hue, 0.5f, 0.5f);
        return new Color(rgb);
    }

} // END CLASS
