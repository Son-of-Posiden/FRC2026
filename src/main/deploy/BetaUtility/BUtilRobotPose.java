// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package RobotPositionUtility;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class RobotPositionUtility {

    //Input the robot pose and the target pose, and get back the radians of how far off the robot is from the target.
    //Returns negative and positive values
    public double getToTargetRadians(Pose2d robotPose, Translation2d targetPose) {

        // get the differenece between the target and the robot
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        // find the angle from the bot to the target
        double fieldRelativeRadians = Math.atan2(dy, dx);

        // factor in the gyro
        double robotRelativeRadians = MathUtil.angleModulus(fieldRelativeRadians - robotPose.getRotation().getRadians());
        return robotRelativeRadians;
    }
    
}
