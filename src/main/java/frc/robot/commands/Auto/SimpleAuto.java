package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.subsystems.SwerveDrive;

import java.util.HashMap;

public class SimpleAuto {

    public static Command getAuto(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("path", new PathConstraints(3, 3));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        return new PathFactory(drive, path, eventMap,true).getCommand();
    }
    public static Command getSpinAuto(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("Spin", new PathConstraints(3, 3));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        return new PathFactory(drive, path, eventMap,true).getCommand();
    }

    public static Command getSquareAuto(SwerveDrive drive){
        PathPlannerTrajectory path = PathPlanner.loadPath("square", new PathConstraints(3, 3));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        return new PathFactory(drive, path, eventMap,true).getCommand();
    }



}
