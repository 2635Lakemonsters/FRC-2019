/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.model;

import java.io.File;
import java.io.FilenameFilter;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.PathCommand;

/**
 * Add your docs here.
 */
public class AutonomousLibrary {

    public static CommandGroup CenterLeftToCargoLeft()
    {
        CommandGroup output = new CommandGroup();

        PathCommand pathCmd = new PathCommand("CenterLeftToCargoLeft", false);
        output.addSequential(pathCmd);

        return output;
    }
    public static CommandGroup CornerLeftToCargoLeft() {
        CommandGroup output = new CommandGroup();

        PathCommand pathCmd = new PathCommand("CornerLeftToCargoLeft", false);
        output.addSequential(pathCmd);

        return output;
    }

    
  /**
   * This function inspects the "deploy/paths" folder and builds
   * a list of commands for each Path it finds. This can be used to 
   * dynamically build a chooser.
   */
    public static Map<String, Command>  GetPathCommands()
    {
        
        Map<String, Command> pathCommands = new HashMap<String, Command>();

        
        File dir = new File(Filesystem.getDeployDirectory(), "paths");
        File [] files = dir.listFiles(new FilenameFilter() {
            @Override
            public boolean accept(File dir, String name) {
                return name.endsWith(".right.pf1.csv");
            }
        });

        for (File pathFile : files) {
           String pathName = pathFile.getName().replace(".right.pf1.csv", "");
           pathCommands.put(pathName,  new PathCommand(pathName, false));
           
        }

        return pathCommands;
       
    }
}
