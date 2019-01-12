 package main.java.frc.robot.model;



public class PathDatum {
	//dt,x,y,position,velocity,acceleration,jerk,heading
	public double dt;
	public double x;
	public double y;
    public double position;
    public double veloicity;
    public double acceleration;
    public double jerk;
    public double heading;

    public void Init(String lineStr)  {
        string[] pathDatumStr = lineStr.split(",");
         dt = parseDouble(pathDatumStr[0]);
         x = parseDouble(pathDatumStr[1]);
         y = parseDouble(pathDatumStr[2]);
         position = parseDouble(pathDatumStr[3]);
         acceleration = parseDouble(pathDatumStr[4]);
         jerk = parseDouble(pathDatumStr[5]);
         heading = parseDouble(pathDatumStr[6]);
    }

}