/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;


/**
 * Add your docs here.
 */
public class EncoderResetCommand extends TimedCommand {
  /**
   * Add your docs here.
   */
  boolean isStalled;
  int elapsedTimeCount;
  public EncoderResetCommand(double timeout) {
    super(timeout);
    requires(Robot.driveSubsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    elapsedTimeCount = 0;
    isStalled = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      elapsedTimeCount = elapsedTimeCount + 1;
      Robot.driveSubsystem.tankDrive(0.2, 0.2);
      System.out.println("current:" + Robot.driveSubsystem.FRMotor.getOutputCurrent());

      // ignore initial motor current by delaying for elapsedTimeCount
      // otherwise, current test will be tripped immediately
      // elapsedTimeCount = 100 ==> 2 seconds
      if( (Robot.driveSubsystem.FRMotor.getOutputCurrent() > 2.0) && (elapsedTimeCount > 100) ) {
        System.out.println("current > 2");
        isStalled = true;
        
      }
  }

  // Called once after timeout
  @Override
  protected void end() {
    // shutdown motor and reset encoder
    Robot.driveSubsystem.tankDrive(0,0);
    Robot.driveSubsystem.FRMotor.setSelectedSensorPosition(0, 0, 0);
  }

  protected boolean isFinished() {
    // interrupt timed command if motor has stalled
    if (isStalled) {
      return true;
    }
    else {
      return isTimedOut();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
