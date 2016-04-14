/*
* @author Sean Lawlor
* @date November 3, 2011
* @class ECSE 211 - Design Principle and Methods
* 
* Modified by F.P. Ferrie
* February 28, 2014
* Changed parameters for W2014 competition

*/
package termproject;

public enum StartCorner {
	BOTTOM_LEFT(1,0,0,0, "BL"),
	BOTTOM_RIGHT(2,Constants.TILE_LENGTH * 10,0,-90, "BR"),
	TOP_RIGHT(3,Constants.TILE_LENGTH * 10,Constants.TILE_LENGTH * 10, 180, "TR"),
	TOP_LEFT(4,0,Constants.TILE_LENGTH * 10, 90, "TL"),
	NULL(0,0,0,0, "NULL");
	
	private int id;
	private double x, y, th;
	private String name;
	private StartCorner(int id, double x, double y, double th, String name) {
		this.id = id;
		this.x = x;
		this.y = y;
		this.th = th;
		this.name = name;
	}
	
	public String toString() {
		return this.name;
	}
	
	public double[] getCooridinates() {
		return new double[] {this.x, this.y};
	}
	
	public double getX() {
		return this.x;
	}
	
	public double getY() {
		return this.y;
	}
	
	public double getTh() {
		return this.th;
	}
	
	public int getId() {
		return this.id;
	}
	
	public static StartCorner lookupCorner(int cornerId) {
		for (StartCorner corner : StartCorner.values())
			if (corner.id == cornerId)
				return corner;
		return NULL;
	}
}
