/**
 * Interface for controller of the ultrasonic sensor
 * @author Karine Mellata and Mustafa Khawaja
 *
 */
public interface UltrasonicController {
	
	public void processUSData(int distance);
	public int readUSDistance();
}
