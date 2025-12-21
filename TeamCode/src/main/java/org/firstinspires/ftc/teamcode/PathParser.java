package org.firstinspires.ftc.teamcode;

import android.content.res.XmlResourceParser;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.xmlpull.v1.XmlPullParser;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PathParser {
    public static abstract class PathAction {
        public int id;
    }

    public static class PointAction extends PathAction {
        public double speed;
        public double x;
        public double y;
        public double rx;
    }

    public static class LaunchAction extends PathAction {
        public int count;
    }

    public static class DrivePath {
        public String alliance; // "RED" or "BLUE"
        public List<PathAction> actions = new ArrayList<>();
    }

    public static DrivePath parse(XmlResourceParser parser, String alliance, Telemetry telemetry) {
        List<DrivePath> drivePaths = new ArrayList<>();
        DrivePath currentPath = null;
        try {
            int eventType = parser.getEventType();

            while (eventType != XmlPullParser.END_DOCUMENT) {

                if (eventType == XmlPullParser.START_TAG) {
                    String tagName = parser.getName();

                    switch (tagName) {

                        case "path":
                            currentPath = new DrivePath();
                            currentPath.alliance = parser.getAttributeValue(null, "alliance");
                            drivePaths.add(currentPath);
                            telemetry.addData("Found path", "Alliance: %s",
                                    currentPath.alliance);
                            break;

                        case "point":
                            if (currentPath != null) {
                                PointAction point = new PointAction();
                                point.id = getInt(parser, "id");
                                point.speed = getDouble(parser, "speed");
                                point.x = getDouble(parser, "x");
                                point.y = getDouble(parser, "y");
                                point.rx = getDouble(parser, "rx");
                                currentPath.actions.add(point);
                                telemetry.addData("Found point", "X: %f Y: %f RX: %f " +
                                        "Speed: %f", point.x, point.y, point.rx, point.speed);
                            }
                            break;

                        case "launch":
                            if (currentPath != null) {
                                LaunchAction launch = new LaunchAction();
                                launch.id = getInt(parser, "id");
                                launch.count = getInt(parser, "count");
                                currentPath.actions.add(launch);
                                telemetry.addData("Found launch", "Count: %d",
                                        launch.count);
                            }
                            break;
                    }
                }

                eventType = parser.next();
            }
        } catch (Exception e) {
            telemetry.addData("Error while parsing", "%s: %s",
                    e, e.getMessage());
        }
        parser.close();
        Optional<DrivePath> dp_opt = drivePaths.stream().filter(
                drivePath -> drivePath.alliance.equals(alliance)
        ).findFirst();
        return dp_opt.orElse(null);
    }

    private static int getInt(XmlResourceParser parser, String attr) {
        return Integer.parseInt(parser.getAttributeValue(null, attr));
    }

    private static double getDouble(XmlResourceParser parser, String attr) {
        return Double.parseDouble(parser.getAttributeValue(null, attr));
    }
}
