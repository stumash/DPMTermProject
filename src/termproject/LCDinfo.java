package termproject;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.utility.TimerListener;

public class LCDinfo implements TimerListener{
	private Odometer odo;
	private TextLCD LCD = LocalEV3.get().getTextLCD();;
	private double[] pos;
	
	public LCDinfo(Odometer odo) {
		this.odo = odo;
		pos = new double [3];
	}
	
	public void timedOut() { 
		odo.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawInt((int)Math.round((pos[0])), 3, 0);
		LCD.drawInt((int)Math.round((pos[1])), 3, 1);
		LCD.drawInt((int)Math.round(fixAngleDeg(Math.toDegrees(pos[2]))), 3, 2);
	}
	
	private double fixAngleDeg(double deg) {
		deg %= 360.0;
		if (deg < 0.0) {
			deg += 360;
		}
		return deg;
	}
}
