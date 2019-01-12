/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import main.java.frc.robot.model.PathDatum;;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX FRMotor;
  WPI_TalonSRX BRMotor;
  WPI_TalonSRX FLMotor;
  WPI_TalonSRX BLMotor;

  PathDatum[] DrivePath;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public DriveSubsystem(){
    FRMotor = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR_CHANNEL);
    BRMotor = new WPI_TalonSRX(RobotMap.BACK_RIGHT_MOTOR_CHANNEL);
    FLMotor = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR_CHANNEL);
    BLMotor = new WPI_TalonSRX(RobotMap.BACK_LEFT_MOTOR_CHANNEL);

    BRMotor.follow(FRMotor);
    BLMotor.follow(FLMotor);
  }

  public void driveLoop(){
    double leftValue;
    double rightValue;

    leftValue = -Robot.oi.leftJoy.getRawAxis(1);
    rightValue = Robot.oi.rightJoy.getRawAxis(1);

    FRMotor.set(rightValue);
    FLMotor.set(leftValue);
  }

  public void LoadPath(String pathFile) throws IOException  {
     //File file = new File(pathFile);

    
      FileReader fileReader = new FileReader(pathFile);
      BufferedReader bufferedReader = new BufferedReader(fileReader);
      List<String> lines = new ArrayList<String>();
      String line = null;
      while ((line = bufferedReader.readLine()) != null) {
          lines.add(line);
      }
      bufferedReader.close();
      String[] pathInfo = lines.toArray(new String[lines.size()]);
      DrivePath = new PathDatum[lines.size()];

      int arrayIndex = 0;
      for (String pathLine : pathInfo) {

        PathDatum pt = new PathDatum();
        pt.Init(pathLine);
        DrivePath[arrayIndex] = pt;

        arrayIndex++;
      }

      

  }
}
