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
  public Position currentPosition;
  double initialEncoderPosition;

  public Switcher() {
    switchMotor = new CANSparkMax(RobotMap.SWITCH_MOTOR_CHANNEL, MotorType.kBrushless);
    controller = new CANPIDController(switchMotor);
    encoder = new CANEncoder(switchMotor);
    encoderStart();
    //Does this effectively reset the encoder???
    //switchMotor.setParameter(ConfigParameter.kEncoderSampleDelta, 0);
  }
  public void encoderStart() {
    controller.setP(1.0);
    controller.setI(0.0);
    controller.setD(0.0);
    controller.setFF(0.0);
  }

  public void encoderReset() {
    this.initialEncoderPosition = encoder.getPosition();
    this.currentPosition = Position.FLOOR;
  }

  public void moveSwitch(Position setPoint) {
    //PLACE LOGIC HEAR FOR WHEN ABLE TO GO TO WHAT STATE?!
    //if elevator at top, may go to rear
    //if elevator at bottom, may not change state
    //Probably only go to floor level if elevator at bottom
    controller.setReference(setPoint.position+initialEncoderPosition, ControlType.kPosition);
    this.currentPosition = setPoint;
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static enum Position {
    FLOOR(RobotMap.SWITCHER_FLOOR),
    CARGO(RobotMap.SWITCHER_CARGO),
    HATCH(RobotMap.SWITCHER_HATCH),
    REAR(RobotMap.SWITCHER_REAR);

    public double position;
    private Position(double position) {
      this.position = position;
    }
  }

  public Position getNextPosition(){
    switch(currentPosition){
      case FLOOR:
        return Position.CARGO;
      case CARGO:
        return Position.HATCH;
      case HATCH:
        return Position.REAR;
      case REAR:
        return Position.REAR;
      default:
        return Position.FLOOR;
    }
  }

  public Position getPrevPosition(){
    switch(currentPosition){
      case FLOOR:
        return Position.FLOOR;
      case CARGO:
        return Position.FLOOR;
      case HATCH:
        return Position.CARGO;
      case REAR:
        return Position.HATCH;
      default:
        return Position.FLOOR;
    }
  }
}

