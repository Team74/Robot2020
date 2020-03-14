package frc.lib.spline;

import java.util.ArrayList;
import java.util.List;

import frc.lib.utils.geometry.*;

public class SplineGenerator {

    private static final double kMaxDX = 2.0;//Inches
    private static final double kMaxDY = 0.05;//Inches
    private static final double kMaxDTheta= 0.1;//Radians
    private static final int kMinSampleSize = 1;
    
    /**
     * Parameterizes ONE Spline
     * @param spline spline to parameterize
     * @param maxDx Maximum change in x allowed in the spline
     * @param maxDy Maximum change in y allowed in the spline
     * @param maxDTheta Maximum change in heading allowed in the spline
     * @param t0 Start time of the spline, generally between 0 and 1
     * @param t1 End time of the spline, generally between  0 and 1
     * @return
     */
    public static List<Pose2dWithCurvature> parameterizeSpline(Spline _spline, double _maxDx, double _maxDy, double _maxDTheta, double _t0, double _t1) {
		List<Pose2dWithCurvature> rv = new ArrayList<Pose2dWithCurvature>();
        rv.add(_spline.getPose2dWithCurvature(0.0));//Add start position
        double dt = (_t1 - _t0);//Get length of time interval to parameterize
        return rv;
    }

    /*
        Convenience function to parametrize a spline with some variables held constant
    */
    //Run one spline with constant t from 0-1, kMaxDX, kMaxDY, kMaxDTheta
    public static List<Pose2dWithCurvature> parameterizeSpline(Spline _s) {
        return parameterizeSpline(_s, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
    }

    /**
     * Run one spline constant t from 0-1 and everything else specified
     * @param s
     * @param maxDx
     * @param maxDy
     * @param maxDTheta
     * @return
     */
    public static List<Pose2dWithCurvature> parameterizeSpline(Spline _s, double _maxDx, double _maxDy, double _maxDTheta) {
        return parameterizeSpline(_s, _maxDx, _maxDy, _maxDTheta, 0.0, 1.0);
    }

    //Run multiple splines with Constant t from 0-1, kMaxDX, kMaxDY, kMaxDTheta
    public static List<Pose2dWithCurvature> parameterizeSplines(List<Spline> splines) {
        return parameterizeSplines(splines, kMaxDX, kMaxDY, kMaxDTheta);
    }

    //Run multiple splines from t 0-1 with everything else specified.
    public static List<Pose2dWithCurvature> parameterizeSplines(List<? extends Spline> splines, double maxDx, double maxDy,
                                                                double maxDTheta) {
        List<Pose2dWithCurvature> rv = new ArrayList<>();
        if (splines.isEmpty()) return rv;
        rv.add(splines.get(0).getPose2dWithCurvature(0.0));
        for (final Spline spline : splines) {
            List<Pose2dWithCurvature> samples = parameterizeSpline(spline, maxDx, maxDy, maxDTheta);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    //Generate arc in between segments of a spline
    private static void getSegmentArc(Spline s, List<Pose2dWithCurvature> rv, double t0, double t1, double maxDx, double maxDy, double maxDTheta) {
        Translation2d p0 = s.getPoint(t0);
        Translation2d p1 = s.getPoint(t1);
        Rotation2d r0 = s.getHeading(t0);
        Rotation2d r1 = s.getHeading(t1);
        Pose2d transformation = new Pose2d(new Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
        Twist2d twist = Pose2d.log(transformation);
        if (twist.dy > maxDy || twist.dx > maxDx || twist.dtheta > maxDTheta) {
            getSegmentArc(s, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
            getSegmentArc(s, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        } else {
            rv.add(s.getPose2dWithCurvature(t1));
        }
    }
}