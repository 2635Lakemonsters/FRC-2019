/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Cargo extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  WPI_TalonSRX leftMotor;
  WPI_TalonSRX rightMotor;

  public Cargo(){
    leftMotor = new WPI_TalonSRX(9);
    rightMotor = new WPI_TalonSRX(8);
  }

  public void cargoOut(){
    control(0.6, 0.6);
  }

  public void cargoIn(){
    control(-1, -1);
  }

  public void cargoStop(){
    control(0,0);
  }

  public void control(double left, double right){
    leftMotor.set(-left);
    rightMotor.set(right);
  }
}
