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
  public CANSparkMax switchMotor;
  public CANPIDController controller;
  CANEncoder encoder;
  Position currentPosition;
  double initialEncoderPosition;

  public Switcher() {
    switchMotor = new CANSparkMax(RobotMap.SWITCH_MOTOR_CHANNEL, MotorType.kBrushless);
    controller = new CANPIDController(switchMotor);
    encoder = new CANEncoder(switchMotor);
    encoderStart();
  }
  public void encoderStart() {
    controller.setP(1.0);
    controller.setI(0.0);
    controller.setD(0.0);
    controller.setFF(0.0);
  }

  public void encoderReset() {
    this.initialEncoderPosition = encoder.getPosition();
  }

  public void moveSwitch(Position setPoint) {
    controller.setReference(setPoint.position+initialEncoderPosition, ControlType.kPosition);
    this.currentPosition = setPoint;
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static enum Position {
    CARGO(RobotMap.SWITCHER_CARGO),
    HATCH(RobotMap.SWITCHER_HATCH),
    REAR(RobotMap.SWITCHER_REAR);

    public double position;
    private Position(double position) {
      this.position = position;
    }
  }
}

