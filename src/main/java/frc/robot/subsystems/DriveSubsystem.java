/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.model.PathDatum;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
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
  Solenoid gearBoxSolenoid;

  //---------EXPERIMENTAL PATH WEAVER CODE----------
  Encoder m_left_encoder;
  Encoder m_right_encoder;

  AnalogGyro m_gyro;

  EncoderFollower m_left_follower;
  EncoderFollower m_right_follower;
  
  Notifier m_follower_notifier;
  //-----------------------------------------------

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

    

    gearBoxSolenoid = new Solenoid(7);

    BRMotor.follow(FRMotor);
    BLMotor.follow(FLMotor);

    //----EXPERIMENTAL PATH WEAVER CODE----------------
    m_left_encoder = new Encoder(RobotMap.k_left_encoder_port_a, RobotMap.k_left_encoder_port_b);
    m_right_encoder = new Encoder(RobotMap.k_right_encoder_port_a, RobotMap.k_right_encoder_port_b);
    m_gyro = new AnalogGyro(RobotMap.k_gyro_port);
    //-------------------------------------------
  }

  public void driveLoop(){
    double leftValue;
    double rightValue;

    leftValue = -Robot.oi.leftJoy.getRawAxis(1);
    rightValue = Robot.oi.rightJoy.getRawAxis(1);

    FRMotor.set(rightValue);
    FLMotor.set(leftValue);
  }
  //---------EXPERIMENTAL PATH WEAVER CODE-----------
  

  public void ExperimentalPathAutoInit() {

    //System.out.println("Test Out");
    //File tmep = new File(Filesystem.getDeployDirectory(), "paths/" + "Straight.left" + ".pf1.csv");
    //File tmep = new File("/home/lvuser/deploy/paths/Straight.left.pf1.csv");
    //Trajectory feu = new Trajectory(PathfinderJNI.trajectoryDeserializeCSV(tmep.getAbsolutePath()));
    //System.out.println(tmep.exists());
    //System.out.println(Filesystem.getDeployDirectory().toString() + "/paths/" + "Straight.left" + ".pf1.csv");
    Trajectory left_trajectory = PathfinderFRC.getTrajectory("Straight.left"); //change this depending on which side
    Trajectory right_trajectory = PathfinderFRC.getTrajectory("Straight.right");
    //System.out.println("TestEnd");
    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder(m_left_encoder.get(), RobotMap.k_ticks_per_rev, RobotMap.k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / RobotMap.k_max_velocity, 0);

    m_right_follower.configureEncoder(m_right_encoder.get(), RobotMap.k_ticks_per_rev, RobotMap.k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / RobotMap.k_max_velocity, 0);
    
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
  }

  public void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate(m_left_encoder.get());
      double right_speed = m_right_follower.calculate(m_right_encoder.get());
      double heading = m_gyro.getAngle();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      FRMotor.set(left_speed + turn);
      FLMotor.set(right_speed - turn);
    }
  }
  public void endPath() {
    m_follower_notifier.stop();
    FRMotor.set(0);
    FLMotor.set(0);
  }

//-----------------------------------------------------------------------------
  public void PathInit()
  {
    startTime = System.currentTimeMillis();
    index = 0;
    reset();
    FRMotor.configMotionCruiseVelocity(200, 0);
		FLMotor.configMotionCruiseVelocity(200, 0);

		FRMotor.configMotionAcceleration(100, 0);
		FLMotor.configMotionAcceleration(100, 0);
    //FLMotor.configMotionCruiseVelocity(sensorUnitsPer100ms)
    //FRMotor.configMotionCruiseVelocity
  }
  public void reset(){
    FRMotor.setSelectedSensorPosition(0, 0, 0);
    FLMotor.setSelectedSensorPosition(0, 0, 0);
    
    FLMotor.set(ControlMode.PercentOutput, 0);
    FRMotor.set(ControlMode.PercentOutput, 0);
  }

  public void MotionMagic() {
    if(index<RightDrivePath.length) {
      System.out.println("position: "+RightDrivePath[index].position);
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
      lines.remove(0);
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
      lines.remove(0);
      String[] pathInfo = lines.toArray(new String[lines.size()]);
      RightDrivePath = new PathDatum[lines.size()];

      int arrayIndex = 0;
      for (String pathLine : pathInfo) {

        PathDatum pt = new PathDatum();
        pt.Init(pathLine);
        RightDrivePath[arrayIndex] = pt;

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

 boolean bullyMode = true;
 public void switchDriveMode(){
  /*
  This function is for changing in and out of buly mode on the 2018 robot. 
  This code may or may not need to be removed on the final robot if we decide to have another bull mode this year
  */

  bullyMode = !bullyMode;
  gearBoxSolenoid.set(bullyMode);
 }

 public void bullyOff(){
   bullyMode= false;

   gearBoxSolenoid.set(bullyMode);
 }

}
