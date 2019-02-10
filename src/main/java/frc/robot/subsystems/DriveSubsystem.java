/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;
import frc.robot.model.PathDatum;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;

import com.ctre.phoenix.motorcontrol.*;

import java.io.*;
import java.util.*;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX FRMotor;
  WPI_TalonSRX BRMotor;
  public WPI_TalonSRX FLMotor;
  WPI_TalonSRX BLMotor;
  public boolean isReversed = false;

  public long startTime;
  int  index;
  PathDatum[] LeftDrivePath;
  PathDatum[] RightDrivePath;
  Solenoid gearBoxSolenoid;

  //---------EXPERIMENTAL PATH WEAVER CODE----------
  // Encoder m_left_encoder;
  // Encoder m_right_encoder;

  //AnalogGyro m_gyro;
  AHRS m_gyro;

  public EncoderFollower m_left_follower;
  public EncoderFollower m_right_follower;
  
  Notifier m_follower_notifier;
  //-----------------------------------------------

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());
  }

  public DriveSubsystem(){
    FRMotor = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR_CHANNEL);
    BRMotor = new WPI_TalonSRX(RobotMap.BACK_RIGHT_MOTOR_CHANNEL);
    FLMotor = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR_CHANNEL);
    BLMotor = new WPI_TalonSRX(RobotMap.BACK_LEFT_MOTOR_CHANNEL);

    FRMotor.setSensorPhase(false);
    FLMotor.setSensorPhase(true);

    //2018 Bot = Right:true   Left:false
    

    gearBoxSolenoid = new Solenoid(7);

    BRMotor.follow(FRMotor);
    BLMotor.follow(FLMotor);
    
    
    //----EXPERIMENTAL PATH WEAVER CODE----------------
    // m_left_encoder = new Encoder();
    // m_right_encoder = new Encoder();
    m_gyro = new AHRS(I2C.Port.kMXP);
    
    //m_gyro = new AnalogGyro(RobotMap.k_gyro_port);
    //-------------------------------------------
  }

  public void setReverse(boolean isReversed) {
      this.isReversed = isReversed;
      if(isReversed) {

  
        FRMotor.setSensorPhase(true);
        FLMotor.setSensorPhase(false);
      } else {
        FRMotor.setSensorPhase(false);
        FLMotor.setSensorPhase(true);
      }

  }

  public boolean getReverse() {
    return isReversed;
  }

  public double leftValue;
  public double rightValue;

  // public void driveLoop(){
  //   leftValue = -Robot.oi.leftJoy.getRawAxis(1);
  //   rightValue = Robot.oi.rightJoy.getRawAxis(1);
  // }

  public void tankDrive(double left, double right) {
    double tempLeft;
		double absleft = Math.abs(left);
		double absright = Math.abs(right);
		if(absleft<0.05) left = 0;
    if(absright<0.05) right = 0;
    if(getReverse()) {
      tempLeft = right;
      right = -left;
      left = -tempLeft;
    } 
    FRMotor.set(-right);
    FLMotor.set(left);
    //System.out.println("Left: " + FLMotor.getSelectedSensorPosition());
    //System.out.println("Right: " + FRMotor.getSelectedSensorPosition());
			
		}
  //---------EXPERIMENTAL PATH WEAVER CODE-----------
  
  
  public void PathAutoInit() {
    FRMotor.setSelectedSensorPosition(0, 0, 0);
    FLMotor.setSelectedSensorPosition(0, 0, 0);
    
    //System.out.println("Test Out");
    //File tmep = new File(Filesystem.getDeployDirectory(), "paths/" + "Straight.left" + ".pf1.csv");
    //File tmep = new File("/home/lvuser/deploy/paths/Straight.left.pf1.csv");
    //Trajectory feu = new Trajectory(PathfinderJNI.trajectoryDeserializeCSV(tmep.getAbsolutePath()));
    //System.out.println(tmep.exists());
    //System.out.println(Filesystem.getDeployDirectory().toString() + "/paths/" + "Straight.left" + ".pf1.csv");
   // Trajectory left_trajectory = PathfinderFRC.getTrajectory("Curved.right"); //change this depending on which side
    //Trajectory right_trajectory = PathfinderFRC.getTrajectory("Curved.left");
    //System.out.println("TestEnd");
    Trajectory left_trajectory;
    Trajectory right_trajectory;
    
    left_trajectory = PathfinderFRC.getTrajectory("DriveStraight.right"); //change this depending on which side
    right_trajectory = PathfinderFRC.getTrajectory("DriveStraight.left");
    
   
    
    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder(FLMotor.getSelectedSensorPosition(0), RobotMap.k_ticks_per_rev, RobotMap.k_wheel_diameter);
    // You must tune the PID values on the following line!
    //m_left_follower.configurePIDVA(0.1, 0.0, 0.25, 1 / RobotMap.k_max_velocity, 0); //1 / RobotMap.k_max_velocity
    m_left_follower.configurePIDVA(0.2, 0.0, 0.08, 0.075, 0.045); //1 / RobotMap.k_max_velocity

    m_right_follower.configureEncoder(FRMotor.getSelectedSensorPosition(0), RobotMap.k_ticks_per_rev, RobotMap.k_wheel_diameter);
    // You must tune the PID values on the following line!
    //m_right_follower.configurePIDVA(0.1, 0.0, 0.25, 1 / RobotMap.k_max_velocity, 0);
    m_right_follower.configurePIDVA(0.2, 0.0, 0.08,0.075, 0.045);
    m_gyro.reset();
    
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
    //System.out.println("Dt: " + left_trajectory.get(0).dt);
  }

  public void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      endPath();
    } else {
      double left_speed = m_left_follower.calculate(FLMotor.getSelectedSensorPosition(0));
      double right_speed = m_right_follower.calculate(FRMotor.getSelectedSensorPosition(0));
      
      //double heading = m_gyro.getAngle();
      double heading = getGyroAngle();
      double desired_heading;
      
      desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      //double turn =  1.5 * (-1.0/80.0) * heading_difference;
      double turn =  0.02 * heading_difference;
      // FRMotor.set(ControlMode.PercentOutput, -(right_speed) );
      // FLMotor.set(ControlMode.PercentOutput, left_speed);
      // if(!swapped){
         //FRMotor.set(ControlMode.PercentOutput, -(right_speed + turn) );
         //FLMotor.set(ControlMode.PercentOutput, left_speed - turn);
         tankDrive(left_speed + turn, (right_speed - turn));
      // }else{
      //   FRMotor.set(ControlMode.PercentOutput, (right_speed + turn) );
      //   FLMotor.set(ControlMode.PercentOutput, -(left_speed - turn));
      // }
      // //System.out.println("Controller Position: " + right_speed + " Left: " + left_speed);
      //System.out.println("Gyro heading: " + heading + " Heading Diff: " + heading_difference);
      //System.out.println(left_speed);
      
    }
  }
  public double getGyroAngle() {
    double angle = m_gyro.getAngle();
    // if(isReversed) {
    //   angle = -angle;
    // }
    return angle;
  }

  
  public void endPath() {
    
    m_follower_notifier.stop();
    tankDrive(0.0,0.0);
    //FRMotor.set(0);
    //FLMotor.set(0);
  }

//-----------------------------------------------------------------------------
 

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
