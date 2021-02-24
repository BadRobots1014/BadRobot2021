/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * Add your docs here.
 */
public class RamseteUtil {
    public static Command getRamseteCommandForPath(String pathWeatherJsonPath, DriveTrainSubsystem driveTrain) {
        Trajectory trajectory = null;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathWeatherJsonPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathWeatherJsonPath, ex.getStackTrace());
            return new WaitCommand(1);
        }    

        RamseteCommand ramseteCommand = new RamseteCommand(trajectory,
         driveTrain::getPose,
         new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
         driveTrain.getDriveKinematics(), 
         driveTrain::setSpeeds, 
         driveTrain);
        
        return ramseteCommand;
    }
}
