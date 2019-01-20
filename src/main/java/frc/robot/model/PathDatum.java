 package frc.robot.model;



public class PathDatum {
	//dt,x,y,position,velocity,acceleration,jerk,heading
	public float dt;
	public float x;
	public float y;
    public float position;
    public float veloicity;
    public float acceleration;
    public float jerk;
    public float heading;

    public void Init(String lineStr)  {
        String[] pathDatumStr = lineStr.split(",");
        if (pathDatumStr.length == 8){
        //System.out.println("dt: "+ dt);
         dt = Float.valueOf(pathDatumStr[0]);
         x = Float.valueOf(pathDatumStr[1]);
         y = Float.valueOf(pathDatumStr[2]);
         position = Float.valueOf(pathDatumStr[3]);
         acceleration = Float.valueOf(pathDatumStr[4]);
         jerk = Float.valueOf(pathDatumStr[5]);
         heading = Float.valueOf(pathDatumStr[6]);
        }
    }

}