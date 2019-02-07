/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Switcher extends Subsystem {
  // Put methods for controlling this subsystem
  CANSparkMax switchMotor;
  CANPIDController controller;

  public Switcher() {
    switchMotor = new CANSparkMax(RobotMap.SWITCH_MOTOR_CHANNEL, MotorType.kBrushless);
    controller = new CANPIDController(switchMotor);

  }
  public void encoderStart() {
    controller.setReference(2000, ControlType.kPosition);
    controller.setP(1.0);
    controller.setI(0.0);
    controller.setD(0.0);
    controller.setFF(0.0);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
