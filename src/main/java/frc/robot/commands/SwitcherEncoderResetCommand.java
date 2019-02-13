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
public class SwitcherEncoderResetCommand extends TimedCommand {
  /**
   * Add your docs here.
   */
  boolean isStalled;
  int elapsedTimeCount;
  public SwitcherEncoderResetCommand(double timeout) {
    super(timeout);
    requires(Robot.switcher);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("switch reset init");
    elapsedTimeCount = 0;
    isStalled = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      System.out.println("switch reset execute");

      elapsedTimeCount = elapsedTimeCount + 1;
      Robot.switcher.switchMotor.set(-0.1);
      System.out.println("current:" + Robot.switcher.switchMotor.getOutputCurrent());

      // ignore initial motor current by delaying for elapsedTimeCount
      // otherwise, current test will be tripped immediately
      // elapsedTimeCount = 100 ==> 2 seconds
      if( (Robot.switcher.switchMotor.getOutputCurrent() > 2.0) && (elapsedTimeCount > 100) ) {
        System.out.println("current > 2");
        isStalled = true;
        
      }
  }

  // Called once after timeout
  @Override
  protected void end() {
    // shutdown motor and reset encoder
    Robot.switcher.switchMotor.set(0);
    Robot.switcher.encoderReset();
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
