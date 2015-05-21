package phys2d.panels;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import javax.swing.JPanel;

@SuppressWarnings("serial")
public class AnimatedPane extends JPanel implements Runnable{

	private final Thread animator;
	private boolean isRunning = true;

	protected final int updateRate;
	protected final double dt;
	protected final int maxFramesSkippable;
	protected final int maxFps;
	
	protected double alpha;
	
	/**
	 * Create a new animated pane
	 * @param updateRate The number of physics/numeric updates to be carried out per second
	 * @param maxFps The total number of frames that can be rendered per second. To prevent overuse of system resources.
	 * @param maxFramesSkippable For slow systems, the number of frames that can be sacrificed to conserve accurate calculations
	 */
	public AnimatedPane(int updateRate, int maxFps, int maxFramesSkippable){
		super();
		
		this.setIgnoreRepaint(true);
		this.setDoubleBuffered(true);
		this.setBackground(Color.red);
		
		this.updateRate = updateRate;
		this.maxFps = maxFps;
		this.maxFramesSkippable = maxFramesSkippable;
		this.dt = 1.0 / updateRate; //delay between game updates (NOT RENDERS) in seconds

		animator = new Thread(this);
		animator.start();
	}

	public void endAnimation(){
		this.isRunning = false;
	}

	public void init(){}
	
	public void update(){}

	public void render(Graphics2D g2d){}

	@Override
	public void paintComponent(Graphics g){
		super.paintComponent(g);
		render((Graphics2D)g);
	}
	
	@Override
	public void run() {
		requestFocusInWindow();
		
		double currentTime = System.nanoTime() / 1e9;  //current time in s
		double accumulator = 0.0;
		double timeSinceLastUpdate;
		
		double timeTakenForLoop;
		
		init();
		
		while(isRunning){
			timeSinceLastUpdate = (System.nanoTime() / 1e9) - currentTime;
			currentTime = System.nanoTime() / 1e9;
			
			//Makes sure that the engine doesnt stop if there is a recurring failure to keep time
			if(timeSinceLastUpdate > maxFramesSkippable * dt)  
				timeSinceLastUpdate = maxFramesSkippable * dt;
			
			accumulator += timeSinceLastUpdate;
			int updates = 0;
			while(accumulator >= dt){
				update();
				accumulator -= dt;
				updates++;
			}
			
			alpha = accumulator / dt;  //percentage time remaining after updates. Used for smooth rendering
			 
			repaint();
			
			timeTakenForLoop = (System.nanoTime() / 1e9) - currentTime;
			callSleep((1000.0 / maxFps) - (timeTakenForLoop * 1e3));  //Caps FPS to maxFps
			
		}
		postAnimationCleanUp();
		
		System.out.println("EXECUTION FINISHED");
	}	

	private void postAnimationCleanUp(){}

	/**
	 * Pause the main thread for time milliseconds.
	 * @param time the time in ms to pause the thread.
	 */
	private void callSleep(double time){
		//System.out.println("Sleep time: " + time);
		if (time > 0){
			try {
				Thread.sleep((long)time, (int)Math.round(((time % 1) * 1e6)));
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		else{
			//System.out.println("NO SLEEP");
		}
	}
}
