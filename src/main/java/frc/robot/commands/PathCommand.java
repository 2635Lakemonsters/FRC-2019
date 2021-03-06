/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.subsystems.DriveSubsystem;

public class PathCommand extends Command {
  String pathName;
  boolean isReversed;
  public PathCommand(String pathName, boolean isReversed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
    this.pathName = pathName;
    this.isReversed = isReversed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    System.out.println("PathCommand:Initialize, isReversed: " + isReversed);
    boolean pathInitialized = Robot.driveSubsystem.PathAutoInit(this.pathName);
    if (!pathInitialized)
      end();
    Robot.driveSubsystem.setReverse(isReversed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean followersIsFinished = Robot.driveSubsystem.m_left_follower.isFinished() || Robot.driveSubsystem.m_right_follower.isFinished();
    if (followersIsFinished) {
      interrupted();
      System.out.println("PathCommand finished");
      return followersIsFinished;
    }
    else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveSubsystem.endPath();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }

}
