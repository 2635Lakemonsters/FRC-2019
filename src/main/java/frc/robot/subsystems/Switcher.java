/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator.Height;

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
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public Switcher() {
    switchMotor = new CANSparkMax(RobotMap.SWITCH_MOTOR_CHANNEL, MotorType.kBrushless);
    controller = new CANPIDController(switchMotor);
    encoder = new CANEncoder(switchMotor);
    currentPosition = Position.FLOOR;
    encoderStart();
    //Does this effectively reset the encoder???
    //switchMotor.setParameter(ConfigParameter.kEncoderSampleDelta, 0);
  }
  public void encoderStart() {
    
    // PID coefficients
     kP = 0.1; 
     //kI = 1e-4;
     //kD = 1; 
     kI = 0.0;
     kD = 0.0; 
     kIz = 0; 
     kFF = 0; 
     kMaxOutput = 1; 
     kMinOutput = -1;
     // set PID coefficients
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setIZone(kIz);
    controller.setFF(kFF);
    controller.setOutputRange(kMinOutput, kMaxOutput);

  }

  public void encoderReset() {
    this.initialEncoderPosition = encoder.getPosition();
    System.out.println("Switcher intitial encoder position: " + initialEncoderPosition);
    this.currentPosition = Position.FLOOR;
  }

  public void moveSwitch(Position setPoint) {
    //LOGIC HEAR FOR WHEN ABLE TO GO TO WHAT STATE
    //if elevator at top, may go to rear
    //if elevator at bottom, may not change state
    //Probably only go to floor level if elevator at bottom
    System.out.println("Switcher.moveSwitch");
    controller.setReference(setPoint.position+initialEncoderPosition, ControlType.kPosition);
    this.currentPosition = setPoint;
    // if(Robot.elevator.currentTargetHeight == Height.GROUND){
    //   if((setPoint == Position.FLOOR && currentPosition == Position.CARGO) || (setPoint == Position.CARGO && currentPosition == Position.FLOOR)){
    //     controller.setReference(setPoint.position+initialEncoderPosition, ControlType.kPosition);
    //     this.currentPosition = setPoint;
    //   }
    // }else if(setPoint == Position.REAR){
    //   if(Robot.elevator.currentTargetHeight == Height.LEVEL3H){
    //     controller.setReference(setPoint.position+initialEncoderPosition, ControlType.kPosition);
    //     this.currentPosition = setPoint;
    //   }
    // }else if(Robot.elevator.currentTargetHeight == Height.GROUND){
    //   System.out.println("Tried to move elevator in bottom state");
    // } else{
    //   controller.setReference(setPoint.position+initialEncoderPosition, ControlType.kPosition);
    //     this.currentPosition = setPoint;
    // }
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
    //System.out.println("Switcher.getNextPosition");
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

