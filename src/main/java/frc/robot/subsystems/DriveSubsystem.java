/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.model.PathDatum;
import com.ctre.phoenix.motorcontrol.*;

import java.io.*;
import java.util.*;

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
  public long startTime;
  int  index;
  PathDatum[] LeftDrivePath;
  PathDatum[] RightDrivePath;
 
 
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

  public void PathInit()
  {
    startTime = System.currentTimeMillis();
    index = 0;
  }

  public void MotionMagic() {
    if(index<RightDrivePath.length) {
      FRMotor.set(ControlMode.MotionMagic, RightDrivePath[index].position);
      FLMotor.set(ControlMode.MotionMagic, LeftDrivePath[index].position);
      index++;
    }
  }
  public boolean isDone(){
    return (index>=RightDrivePath.length);
  }

  public void LeftLoadPath(String pathFile) throws IOException  {
     java.io.File file = new java.io.File(pathFile);

    if(file.exists() && file.isFile()) {
      FileReader fileReader = new FileReader(pathFile);
      BufferedReader bufferedReader = new BufferedReader(fileReader);
      List<String> lines = new ArrayList<String>();
      String line = null;
      while ((line = bufferedReader.readLine()) != null) {
          lines.add(line);
      }
      bufferedReader.close();
      String[] pathInfo = lines.toArray(new String[lines.size()]);
      LeftDrivePath = new PathDatum[lines.size()];

      int arrayIndex = 0;
      for (String pathLine : pathInfo) {

        PathDatum pt = new PathDatum();
        pt.Init(pathLine);
        LeftDrivePath[arrayIndex] = pt;

        arrayIndex++;
      }
    } else{
      System.out.println("--Left path file not found: " + pathFile);
    }
  }

  public void RightLoadPath(String pathFile) throws IOException  {
    //File file = new File(pathFile);

     java.io.File file = new java.io.File(pathFile);

     if(file.exists() && file.isFile()) {
      FileReader fileReader = new FileReader(pathFile);
      BufferedReader bufferedReader = new BufferedReader(fileReader);
      List<String> lines = new ArrayList<String>();
      String line = null;
      while ((line = bufferedReader.readLine()) != null) {
          lines.add(line);
      }
      bufferedReader.close();
      String[] pathInfo = lines.toArray(new String[lines.size()]);
      RightDrivePath = new PathDatum[lines.size()];

      int arrayIndex = 0;
      for (String pathLine : pathInfo) {

        PathDatum pt = new PathDatum();
        pt.Init(pathLine);
        LeftDrivePath[arrayIndex] = pt;

        arrayIndex++;
      }
    }else{
      System.out.println("--Right path file not found: " + pathFile);
    }
 }

 public void LoadPath(String leftPathFile, String rightPathFile) throws IOException {
   LeftLoadPath(leftPathFile);
   RightLoadPath(rightPathFile);

 }

}
