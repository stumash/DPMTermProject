package termproject;
/*
* @author Sean Lawlor
* @date November 3, 2011
* @class ECSE 211 - Design Principle and Methods
* 
* Modified by F.P. Ferrie
* February 28, 2014
* Changed parameters for W2014 competition
* 
* Modified by Francois OD
* November 11, 2015
* Ported to EV3 and wifi (from NXT and bluetooth)
* Changed parameters for F2015 competition
*/
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashMap;

//import wifi.StartCorner;
//import wifi.Transmission;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

public class WifiDPM {
	// example call of the transmission protocol
	// The print function is just for debugging to make sure data is received correctly

	// *** INSTRUCTIONS ***
	// There are two variables to set manually on the EV3 client:
	// 1. SERVER_IP: the IP address of the computer running the server application
	// 2. TEAM_NUMBER: your project team number
	
	private static TextLCD LCD = LocalEV3.get().getTextLCD();

	@SuppressWarnings("unused")
	public static HashMap<String, Integer> getWifiData() throws FileNotFoundException {
		WifiConnection conn = null;
		try {
			conn = new WifiConnection(Constants.SERVER_IP, Constants.TEAM_NUMBER);
		} catch (IOException e) {
			LCD.drawString("Connection frrled", 0, 8);
		}
		
		//example use of StartData
		LCD.clear();
		if (conn != null){
			HashMap<String,Integer> t = conn.StartData;
			if (t == null) {
				LCD.drawString("Failed to read transmission", 0, 5);
			} else {
				LCD.drawString("Transmission read", 0, 5);
				LCD.drawString(t.toString(), 0, 6);
			}
			return t;
		} else {
			LCD.drawString("Connection failed", 0, 5);
			return null;
		}
	}
}
