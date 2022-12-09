package com.robotic.assignment2.main;

public class OccupyMaze {
	//Robot orientation
	int heading;
	//Robot Position
	int x;
	int y;
	
	boolean NorthWall;
	boolean EastWall;
	boolean SouthWall;
	boolean WestWall;

	public OccupyMaze(int heading, int x, int y) {
		this.x = x;
		this.y = y;
		this.heading = heading;

	}
	
	public int getX(){
		return x;
	}
	
	public int getY() {
		return y;
	}
	
	
	//North = 1, East = 2, South = 3, West = 4
	public int getHeading() {
		return heading;
	}
	
	public void setHeading(int heading) {
		this.heading = heading;
	}
	
	public void setX(int x) {
		this.x = x;
	}
	
	public void setY(int y) {
		this.y = y;
	}
	
	public void setNorthWall(boolean NorthWall) {
		this.NorthWall= NorthWall;
	}
	
	public boolean getNorthWall() {
		return NorthWall;
	}
	
	public void setEastWall(boolean EastWall) {
		this.EastWall = EastWall;
	}
	
	public boolean getEastWall() {
		return EastWall;
	}
	
	public void setSouthWall(boolean SouthWall) {
		this.SouthWall = SouthWall;
	}
	
	public boolean getSouthWall() {
		return SouthWall;
	}
	
	public void setWestWall(boolean WestWall) {
		this.WestWall = WestWall;
	}
	
	public boolean getWestWall() {
		return WestWall;
	}

}
