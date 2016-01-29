package phys2d;

/**
 * Last edit 210515
 * Eclipse commit test with successful compilation.
 */

import javax.swing.JFrame;

import phys2d.panels.Phys2DPane;

public class Phys2DMain {

    public static JFrame mainPhys2dFrame = new JFrame("Physics 2D");
    public static final int XRES = 1000;
    public static final int YRES = 1000;
    private static final int updateRate = 50;
    private static final int maxFps = 200;
    private static final int maxFramesSkippable = 5;

    public static void main(String[] args) {

        mainPhys2dFrame.setSize(XRES, YRES);
        mainPhys2dFrame.setResizable(false);
        mainPhys2dFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        mainPhys2dFrame.add(new Phys2DPane(updateRate, maxFps, maxFramesSkippable));
        mainPhys2dFrame.setVisible(true);

    }
}
