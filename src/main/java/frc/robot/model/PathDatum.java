 package frc.robot.model;



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
        String[] pathDatumStr = lineStr.split(",");
         dt = Double.parseDouble(pathDatumStr[0]);
         x = Double.parseDouble(pathDatumStr[1]);
         y = Double.parseDouble(pathDatumStr[2]);
         position = Double.parseDouble(pathDatumStr[3]);
         acceleration = Double.parseDouble(pathDatumStr[4]);
         jerk = Double.parseDouble(pathDatumStr[5]);
         heading = Double.parseDouble(pathDatumStr[6]);
    }

}