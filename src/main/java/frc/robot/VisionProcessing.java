package frc.robot;

import java.util.HashMap;

public class VisionProcessing {


    private static final int limelightHorizontalFieldoView = 54;
    private static final int limelightVerticalFieldoView = 41;
    private static final int resolutionHorizontal = 340;
    private static final int resolutionVertical = 240;
    private static final double viewPlaneWidth = 2.0 * Math.tan(limelightHorizontalFieldoView / 2);
    private static final double viewPlaneHeight = 2.0 * Math.tan(limelightVerticalFieldoView / 2);



    /**
     * @param limelightData gives us the pixel of the center of the target, and the height and width of the taret
     * @param targetData gives us the angleof the camera and the height of the center of the target and the height of the camera above the floor.
     * @return A hashmap containing the angle offset from the target and the distance horizontally to the target
     */
    public static HashMap<String, Double> calculateDistanceAngle(HashMap<String, Integer> limelightData, HashMap<String, Double> targetData) {
        if(limelightData.containsKey("targets")) {
            return null;
        }
        double normalizedx = (1/(resolutionHorizontal / 2)) * (limelightData.get("m_x") - (((resolutionHorizontal - 1) / 2)));
        double normalizedy = (1/(resolutionVertical / 2)) * ((((resolutionVertical - 1) / 2)) - limelightData.get("m_y"));
        double viewPlaneX = (viewPlaneWidth / 2) * normalizedx;
        double viewPlaneY = (viewPlaneHeight / 2) * normalizedy;
        double angleX = Math.atan2(1, viewPlaneX);
        double angleY = Math.atan2(1, viewPlaneY);
        double distance = (targetData.get("target_height") - targetData.get("camera_height")) / Math.tan(angleY + targetData.get("camera_angle"));
        final HashMap<String, Double> targetPoseRelative = new HashMap<>(){
            private static final long serialVersionUID = -999;
            {
                put("horizontal_angle", angleX);
                put("distance", distance);
            }
        };
        return targetPoseRelative;
    }
}