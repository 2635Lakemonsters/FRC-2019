/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimberTimeAvailableCommand extends Command {
  public ClimberTimeAvailableCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  final double TIME_TO_CLIMB = 15;
  boolean okToContinue;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double matchTime = Robot.fms.getTime();
    if(TIME_TO_CLIMB < matchTime){
      okToContinue = true;
    }else{
      okToContinue = false;
      Robot.climbCommandGroup.cancel();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
