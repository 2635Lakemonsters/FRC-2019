/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
//import jdk.internal.vm.compiler.collections.EconomicMap;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  WPI_TalonSRX FRExtender;
  WPI_TalonSRX FLExtender;
  WPI_TalonSRX BExtender;
  WPI_TalonSRX BackDriveMotor;

  int EXTENDER_HEIGHT = 0;

  //This int will be used to track what stage the climb was cancelled in, and what to do about that cancel
  public int COMMAND_GROUP_STAGE = 0;

  public Climber() {
    FRExtender = new WPI_TalonSRX(0);
    FLExtender = new WPI_TalonSRX(0);
    BExtender = new WPI_TalonSRX(0);
    BackDriveMotor = new WPI_TalonSRX(0);

    FRExtender.config_kP(0, 0);
    FRExtender.config_kI(0, 0);
    FRExtender.config_kD(0, 0);
    FRExtender.config_kF(0, 0);

  }
 
  public void lowerClimber(){
    FRExtender.set(ControlMode.Position, EXTENDER_HEIGHT);
    FLExtender.set(ControlMode.Position, EXTENDER_HEIGHT);
    BExtender.set(ControlMode.Position, EXTENDER_HEIGHT);

  }public boolean lowerClimberIsFinished(){
    int rightValue = FRExtender.getSelectedSensorPosition(0);
    int leftValue = FLExtender.getSelectedSensorPosition(0);
    int backValue = BExtender.getSelectedSensorPosition(0);

    if(rightValue == EXTENDER_HEIGHT && leftValue == EXTENDER_HEIGHT && backValue == EXTENDER_HEIGHT){
      return true;
    }else{
      return false;
    }
  }

  public void raiseFrontClimber(){
    FRExtender.set(ControlMode.Position, 0);
    FLExtender.set(ControlMode.Position, 0);
    BExtender.set(ControlMode.Position, EXTENDER_HEIGHT);
  }

  public boolean raiseFrontClimberIsFinished(){
    int rightValue = FRExtender.getSelectedSensorPosition(0);
    int leftValue = FLExtender.getSelectedSensorPosition(0);

    if(rightValue == 0 && leftValue == 0){
      return true;
    }else{
      return false;
    }
  }

  public void raiseBackClimber(){
    FRExtender.set(ControlMode.Position, 0);
    FLExtender.set(ControlMode.Position, 0);
    BExtender.set(ControlMode.Position, 0);
  }

  public boolean raiseBackClimberIsFinished(){
    int backValue = BExtender.getSelectedSensorPosition(0);

    if(backValue == 0){
      return true;
    }else{
      return false;
    }
  }

  public void drive(double driveSpeed){
    BackDriveMotor.set(driveSpeed);
  }

  /*
  * One of two options needs to be chosen.
  * Either we use the wall stallout to see when we hit the wall
  * Or we assume we always start directly against the wall, 
  * And use a constant amount of encoder counts to proceed
  */
  public boolean driveHitWall(){
    if( BackDriveMotor.getOutputCurrent() > 2.0) {
      System.out.println("current > 2");
      return true;
    }else{
      return false;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
