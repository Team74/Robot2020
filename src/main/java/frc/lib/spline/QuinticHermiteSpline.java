package frc.lib.spline;

import frc.lib.utils.geometry.Pose2d;
import frc.lib.utils.geometry.Rotation2d;
import frc.lib.utils.geometry.Translation2d;

import java.util.List;

public class QuinticHermiteSpline extends Spline {
    private static final double kEpsilon = 1e-5;
    private static final double kStepSize = 1.0;
    private static final double kMinDelta = 0.001;
    private static final int kSamples = 100;
    private static final int kMaxIterations = 100;

    private double x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1;
    private double ax, bx, cx, dx, ex, fx, ay, by, cy, dy, ey, fy;

    /*
    p0 is start point, p1 is end point
    fairly sure scale undoes the normalization, but not sure about the * 1.2
    ddx and ddy are set to 0 to simplify the math, maybe I'll figure out a way to optimize them
    */ 
    public QuinticHermiteSpline(Pose2d p0, Pose2d p1){
        double scale = 1.2 * p0.getTranslation().distance(p1.getTranslation());
        x0 = p0.getTranslation().x();
        x1 = p1.getTranslation().x();
        dx0 = p0.getRotation().cos() * scale;
        dx1 = p1.getRotation().cos() * scale;
        ddx0 = 0;
        ddx1 = 0;
        y0 = p0.getTranslation().y();
        y1 = p1.getTranslation().y();
        dy0 = p0.getRotation().sin() * scale;
        dy1 = p1.getRotation().sin() * scale;
        ddy0 = 0;
        ddy1 = 0;

        computeCoefficients();
    }

    /**
     * Used by the curvature optimization function
     */
    private QuinticHermiteSpline(double _x0, double _x1, double _dx0, double _dx1, double _ddx0, double _ddx1,
                                 double _y0, double _y1, double _dy0, double _dy1, double _ddy0, double _ddy1) {
        this.x0 = _x0;
        this.x1 = _x1;
        this.dx0 = _dx0;
        this.dx1 = _dx1;
        this.ddx0 = _ddx0;
        this.ddx1 = _ddx1;

        this.y0 = _y0;
        this.y1 = _y1;
        this.dy0 = _dy0;
        this.dy1 = _dy1;
        this.ddy0 = _ddy0;
        this.ddy1 = _ddy1;

        computeCoefficients();
    }

    /*
        Re-arranges the spline into an at^5 + bt^4 + ... + f form for simpler computations
        Further reading on these equations can be found here:
            https://www.google.com/url?q=https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf&source=gmail&ust=1550460099570000&usg=AFQjCNEvPaMOIsWEvbKLQz4gs86-p4QY9Q
    */
    private void computeCoefficients(){
        ax = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
        bx = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
        cx = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
        dx = 0.5 * ddx0;
        ex = dx0;
        fx = x0;

        ay = -6 * y0 - 3 * dy0 - 0.5 * ddy0 + 0.5 * ddy1 - 3 * dy1 + 6 * y1;
        by = 15 * y0 + 8 * dy0 + 1.5 * ddy0 - ddy1 + 7 * dy1 - 15 * y1;
        cy = -10 * y0 - 6 * dy0 - 1.5 * ddy0 + 0.5 * ddy1 - 4 * dy1 + 10 * y1;
        dy = 0.5 * ddy0;
        ey = dy0;
        fy = y0;
    }

    public Pose2d getStartPose() {
        return new Pose2d(
            new Translation2d(x0, y0),
            new Rotation2d(dx0, dy0, true)
        );
    }

    public Pose2d getEndPose() {
        return new Pose2d(
            new Translation2d(x1, y1),
            new Rotation2d(dx1, dy1, true)
        );
    }

    //returns a point on this spline at time t where t is between 0 and 1
    public Translation2d getPoint(double t) {
        double temp_x = ax * Math.pow(t, 5) + bx * Math.pow(t, 4) + cx * Math.pow(t, 3) + dx * Math.pow(t, 2) + ex * t + fx;
        double temp_y = ay * Math.pow(t, 5) + by * Math.pow(t, 4) + cy * Math.pow(t, 3) + dy * Math.pow(t, 2) + ey * t + fy;
        return new Translation2d(temp_x, temp_y);
    }

    //We can get the derivitive at a certain point by deriving the above equations for x and y
    private double dx(double t) {
        return 5 * ax * Math.pow(t, 4) + 4 * bx * Math.pow(t, 3) + 3 * cx * Math.pow(t, 2) + 2 * dx * t + ex;
    }

    private double dy(double t) {
        return 5 * ay *Math.pow(t, 4) + 4 * by * Math.pow(t, 3) + 3 * cy * Math.pow(t, 2) + 2 * dy * t + ey;
    }

    private double ddx(double t) {
        return 20 * ax * Math.pow(t, 3) + 12 * bx * Math.pow(t, 2) + 6 * cx * t + 2 * dx;
    }

    private double ddy(double t) {
        return 20 * ay * Math.pow(t, 3) + 12 * by * Math.pow(t, 2) + 6 * cy * t + 2 * dy;
    }

    private double dddx(double t) {
        return 60 * ax * Math.pow(t, 2) + 24 * bx * t + 6 * cx;
    }

    private double dddy(double t) {
        return 60 * ay * Math.pow(t, 2) + 24 * by * t + 6 * cy;
    }

    public double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }

    public double getCurvature(double t) {
        return (dx(t) * ddy(t) - ddx(t) * dy(t)) / ((dx(t) * dx(t) + dy(t) * dy(t)) * Math.sqrt((dx(t) * dx(t) + dy(t) * dy(t))));
    }

    public  double getDCurvature(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t)*dddy(t) - dddx(t)*dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }

    public double dCurvature2(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t)*dddy(t) - dddx(t)*dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
    }

    //Method to get the heading of the spline at point t
    public Rotation2d getHeading(double t) {
        return new Rotation2d(dx(t), dy(t), true);
    }

    /**
     * @return integral of dCurvature^2 over the length of the spline
     */
    private double sumDCurvature2() {
        double dt = 1.0/kSamples;
        double sum = 0.0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * dCurvature2(t));
        }
        return sum;
    }
        /**
     * @return integral of dCurvature^2 over the length of multiple splines
     */

     public static double sumDCurvature2(List<QuinticHermiteSpline> _splines) {
        double sum = 0.0;
        for(QuinticHermiteSpline s : _splines) {
            sum += s.sumDCurvature2();
        }
        return sum;
     }

    /**
     * Makes optimization code a little more readable
     */
    private static class ControlPoint {
        private double ddx, ddy;
    }

    /**
     * Finds the optimal second derivative values for a set of splines to reduce the sum of the change in curvature
     * squared over the path
     *
     * @param _splines the list of splines to optimize
     * @return the final sumDCurvature2
     */
    public static double optimizeSpline(List<QuinticHermiteSpline> _splines) {
        int count = 0;
        double prev = sumDCurvature2(_splines);
        while (count < kMaxIterations) {
            runOptimizationIteration(_splines);
            double current = sumDCurvature2(_splines);
            if (prev - current < kMinDelta) {
                return current;
            }
            prev = current;
            count++;
        }
        return prev;
    } 
    /**
     * Runs a single optimization iteration
     */
    private static void runOptimizationIteration(List<QuinticHermiteSpline> _splines) {
        //can't optimize anything with less than 2 splines
        if (_splines.size() <= 1) {
            return;
        }

        ControlPoint[] controlPoints = new ControlPoint[_splines.size() - 1];
        double magnitude = 0;

        for (int i = 0; i < _splines.size() - 1; ++i) {
            // Don't try to optimize colinear points
            if (_splines.get(i).getStartPose().isColinear(_splines.get(i + 1).getStartPose()) || _splines.get(i).getEndPose().isColinear(_splines.get(i).getEndPose())) {
                continue;
            }
            double original = sumDCurvature2(_splines);
            QuinticHermiteSpline temp, temp1;

            temp = _splines.get(i);
            temp1 = _splines.get(i + 1);
            controlPoints[i] = new ControlPoint(); //holds the gradient at a control point

            //Calculates partial derivatives of sumDCurvature2
            _splines.set(i, new QuinticHermiteSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0, temp.ddx1 + kEpsilon, 
                                                     temp.y0, temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1));
            _splines.set(i + 1, new QuinticHermiteSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0 + kEpsilon, temp1.ddx1, 
                                                         temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0, temp1.ddy1));
            controlPoints[i].ddx = (sumDCurvature2(_splines) - original) / kEpsilon;
            _splines.set(i, new QuinticHermiteSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0, temp.ddx1,
                                                     temp.y0, temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1 + kEpsilon));
            _splines.set(i + 1, new QuinticHermiteSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0,temp1.ddx1, 
                                                         temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0 + kEpsilon, temp1.ddy1));
            controlPoints[i].ddy = (sumDCurvature2(_splines) - original) / kEpsilon;

            _splines.set(i, temp);
            _splines.set(i + 1, temp1);
            magnitude += Math.pow(controlPoints[i].ddx, 2) + Math.pow(controlPoints[i].ddy, 2);
        }

        magnitude = Math.sqrt(magnitude);

        //minimize along the direction of the gradient
        //first calculate 3 points along the direction of the gradient
        Translation2d p1, p2 , p3;
        p2 = new Translation2d(0, sumDCurvature2(_splines)); //middle point is at current location

        for (int i = 0; i < _splines.size(); ++i) { //first point is offset from the middle location by -stepSize
            if (_splines.get(i).getStartPose().isColinear(_splines.get(i + 1).getStartPose()) || _splines.get(i).getEndPose().isColinear(_splines.get(i + 1).getEndPose())) {
                continue;
            }
            //normalize to stepSize
            controlPoints[i].ddx *= kStepSize / magnitude;
            controlPoints[i].ddy *= kStepSize / magnitude;

            //move opposite the gradient by step size amount
            _splines.get(i).ddx1 -= controlPoints[i].ddx;
            _splines.get(i).ddy1 -= controlPoints[i].ddy;
            _splines.get(i + 1).ddx0 -= controlPoints[i].ddx;
            _splines.get(i + 1).ddy0 -= controlPoints[i].ddy;

            //recompute the spline's coefficients to account for new second derivatives
            _splines.get(i).computeCoefficients();
            _splines.get(i + 1).computeCoefficients();
        }
        p1 = new Translation2d(-kStepSize, sumDCurvature2(_splines));

        for (int i = 0; i < _splines.size(); ++i) { //last point offset from the middle location by +stepSize
            if (_splines.get(i).getStartPose().isColinear(_splines.get(i + 1).getStartPose()) || _splines.get(i).getEndPose().isColinear(_splines.get(i + 1).getEndPose())) {
                continue;
            }
            //move along the gradient by 2 times the step size amount (to return to original location and move by 1 step)
            _splines.get(i).ddx1 += 2 * controlPoints[i].ddx;
            _splines.get(i).ddy1 += 2 * controlPoints[i].ddy;
            _splines.get(i + 1).ddx0 += 2 * controlPoints[i].ddx;
            _splines.get(i + 1).ddy0 += 2 * controlPoints[i].ddy;

            //recompute the spline's coefficients to account for new second derivatives
            _splines.get(i).computeCoefficients();
            _splines.get(i + 1).computeCoefficients();
        }
        p3 = new Translation2d(kStepSize, sumDCurvature2(_splines));

        double stepSize = fitParabola(p1, p2, p3); //approximate step size to minimize sumDCurvature2 along the gradient

        for (int i = 0; i < _splines.size(); ++i) {
            if (_splines.get(i).getStartPose().isColinear(_splines.get(i + 1).getStartPose()) || _splines.get(i).getEndPose().isColinear(_splines.get(i + 1).getEndPose())) {
                continue;
            }
            //move by the step size calculated by the parabola fit (+1 to offset for the final transformation to find p3)
            controlPoints[i].ddx *= 1 + stepSize / kStepSize;
            controlPoints[i].ddy *= 1 + stepSize / kStepSize;

            _splines.get(i).ddx1 += controlPoints[i].ddx;
            _splines.get(i).ddy1 += controlPoints[i].ddy;
            _splines.get(i + 1).ddx0 += controlPoints[i].ddx;
            _splines.get(i + 1).ddy0 += controlPoints[i].ddy;

            //recompute the spline's coefficients to account for new second derivatives
            _splines.get(i).computeCoefficients();
            _splines.get(i + 1).computeCoefficients();               
        }
    }

    /**
     * fits a parabola to 3 points
     *
     * @return the x coordinate of the vertex of the parabola
     */
    private static double fitParabola(Translation2d _p1, Translation2d _p2, Translation2d _p3) {
        double A = (_p3.x() * (_p2.y() - _p1.y()) + _p2.x() * (_p1.y() - _p3.y()) + _p1.x() * (_p3.y() - _p2.y()));
        double B = (_p3.x() * _p3.x() * (_p1.y() - _p2.y()) + _p2.x() * _p2.x() * (_p3.y() - _p1.y()) + _p1.x() * _p1.x() * (_p2.y() - _p3.y()));
        return -B / (2 * A);
    }


}