

import lejos.robotics.SampleProvider;

/**
 * The ultrasonic poller returns the light value read multiplied by 100
 * It sleeps for 50ms at the end of each period.
 * @author Karine Mellata and Mustafa Khawafa
 *
 */

public class UltrasonicPoller extends Thread{
	private SampleProvider us;
	private UltrasonicController cont;
	private float[] usData;
	
	public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicController cont) {
		this.us = us;
		this.cont = cont;
		this.usData = usData;
	}
	
	public void run() {
		int distance;
		while (true) {
			us.fetchSample(usData,0);							// acquire data
			distance=(int)(usData[0]*100.0);						// extract from buffer, cast to int
			cont.processUSData(distance);						// now take action depending on value
			try { Thread.sleep(50); } catch(Exception e){}		// Poor man's timed sampling
		}
	}

}
