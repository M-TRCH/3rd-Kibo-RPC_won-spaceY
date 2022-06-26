
package jp.jaxa.iss.kibo.rpc.defaultapk;
import android.os.SystemClock;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

public class YourService extends KiboRpcService
{
    /* constant variables */
    final Point Pt0     = new Point(10.7615, -6.8849, 5.31674);
    final Point Pt1     = new Point(10.71, -7.7, 4.48);
    // "Pt1" -> original_y = -7.7, shift_y = -7.7722
    final Point Pt1_2   = new Point(10.46, -8.44813, 5.41);
    // "Pt1_2" -> original_z = 5.57, shift_z = 5.41
    final Point Pt2     = new Point(11.2746, -9.92284, 5.29881);
    final Point Pt3     = new Point(11.2746, -7.89178, 4.96538);
    Point Pt_Target = new Point(), Pt_Robot = new Point();
    final Quaternion Qua0 = new Quaternion(0, 0, -0.707f, 0.707f);
    final Quaternion Qua1 = new Quaternion(0, 0.707f, 0, 0.707f);
    final Quaternion Qua2 = new Quaternion(0, 0, -0.707f, 0.707f);
    final Quaternion Qua3 = new Quaternion(0, 0, -0.707f, 0.707f);
    Quaternion Qua_Target = new Quaternion();
    boolean ARFinished = false;


    @Override
    protected void runPlan1()
	{
	    // start mission
	    api.startMission();

        // Target 1
        AimTarget(1);

        // Avoid KOZ (go away)
        moveTo(Pt1_2, Qua2);

        // Target 2
        AimTarget(2);

        // Avoid KOZ (return)
        moveTo(Pt1_2, Qua2);

        // Go to goal position
        moveTo(Pt3, Qua3);
        api.reportMissionCompletion();
	}
    @Override
    protected void runPlan2()
	{
        // Blank space
    }
    @Override
    protected void runPlan3()
    {
        // Blank space
    }
    public void delay(long sleep)
        /* sleep function */ {
        try
        {
            Thread.sleep(sleep);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }
    public void moveTo(Point point, Quaternion quaternion)
        /* Re-check api.moveTo function is successful or until max number of counter. */ {
        Result result;
        int count = 0, max_count = 2;
        do
        {
            result = api.moveTo(point, quaternion, true);
            count++;
        }
        while (!result.hasSucceeded() && count < max_count);
        delay(3000);
    }
    public double[] SrcPointTransformToHomoPoint(Mat H, double[] pt)
        /* Position transformation form source image to homography image */ {
        /* Input ; 3x3 Mat -> Homography constant, 2 double in array -> point x and point y */
        Mat pt1 = new Mat(3, 1, CvType.CV_64FC1);
        Mat	pt2 = new Mat();
        pt1.put(0, 0, pt[0], pt[1], 1);

        Core.gemm(H, pt1, 1, new Mat(), 0, pt2);

        double[] data = pt2.get(2, 0);
        Core.divide(pt2, new Scalar(data[0]), pt2);

        double[] data1 = pt2.get(0, 0);
        double[] data2 = pt2.get(1, 0);
        return new double[]{data1[0], data2[0]};
    }
    public double[] getVector(double[] p,double[] q)
        /* Finding a vector between of position (input two position array). */ {
        return new double[]{p[0]-q[0], p[1]-q[1], p[2]-q[2]};
    }
    public double getVal(double[] p)
        /* Get the value of vector (input array of vector) */ {
        return Math.sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2]));
    }
    public double[] getRoot(double a ,double b , double c)
        /* Get the root of 2 degree equation (input coefficient a, b, c) */ {
        double InSqrt = Math.pow(b, 2)-4*a*c;
        return new double[]
        {
                (-b+Math.sqrt(InSqrt))/2*a,
                (-b-Math.sqrt(InSqrt))/2*a
        };
    }
    public double[] vecCross(double[] p,double[] q)
        /* Cross product of 3D vector (input 2 array of vector) */ {
        return new double[]
        {
            p[1]*q[2]-p[2]*q[1],
            p[2]*q[0]-p[0]*q[2],
            p[0]*q[1]-p[1]*q[0]
        };
    }
    public double[] getQua(double[] p,double[] q)
        /* Convert 2 vector to quaternion (input two array of vector) */{
        double [] n = vecCross(p,q);
        double [] r = {p[0]-q[0],p[1]-q[1],p[2]-q[2]};
        double [] v =
        {
            Math.sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2])),
            Math.sqrt((q[0]*q[0])+(q[1]*q[1])+(q[2]*q[2])),
            Math.sqrt((r[0]*r[0])+(r[1]*r[1])+(r[2]*r[2])),
            Math.sqrt((n[0]*n[0])+(n[1]*n[1])+(n[2]*n[2]))
        };
        double [] u = {n[0]/v[3], n[1]/v[3], n[2]/v[3]};
        double theta =  Math.acos(((v[0]*v[0])+(v[1]*v[1])-(v[2]*v[2]))/(2*v[0]*v[1]));
        return new double[]
        {
            Math.cos(theta/2),
            Math.sin(theta/2)*u[0],
            Math.sin(theta/2)*u[1],
            Math.sin(theta/2)*u[2]
        };
    }
    public void getQuafromVector()
        /* Similar to the moveTo six variables but between is vector ends are pointed by a laser pointer not center of robot. */ {
        double [] robot_pos		= {Pt_Robot.getX(), Pt_Robot.getY(), Pt_Robot.getZ()};
        double [] target_pos	= {Pt_Target.getX(), Pt_Target.getY(), Pt_Target.getZ()};
        double [] target_vec	= getVector(target_pos, robot_pos);
        double target_val		= getVal(target_pos);
        double [] laser_shift	= {0.1302, 0.0572, -0.111} ;
        double laser_val		= getVal (laser_shift);
        double las_angle		= 136.11699 * 0.01745;
        double [] length		= getRoot(1,-2*laser_val*Math.cos(las_angle),(laser_val*laser_val)-(target_val*target_val));

        double laser_length;
        if (length[0] > 0 ){laser_length = length[0];}
        else {laser_length = length[1];}

        double [] origin_vec	= {laser_shift[0]+laser_length, laser_shift[1], laser_shift[2]};
        double [] Qua			= getQua(origin_vec, target_vec);
        Qua_Target = new Quaternion((float)Qua[1], (float)Qua[2], (float)Qua[3], (float)Qua[0]);
    }
    public void AimTarget(int targetNum)
        /* Compute the coordinates of the target position transformed by the homography. */ {

        /* Set AR parameter */
        Mat matSrc = new Mat();						// Variable store matrix of image.
        Mat IDs = new Mat(); 						// Variable store ID each of AR code. : Matrix format
        List<Mat> Corners = new ArrayList<>(); 	// Variable store four corner each of AR Code.
        Dictionary Dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250); // Standard AR code format according rule book.
        int[] AR_ID = new int[]{0, 0, 0, 0};		// Variable store ID each of AR code. : Array of Int format

        /* Set general parameter */
        boolean firstTime = true;
        int loopCount = 0; 				// Variable of loop counter.
        final int loopCountMax = 5;		// Variable of max loop count.

        /* Target tag variables */
        int[]   ArID1_CornerRT = new int[]{0, 0},	// Container the specified corner of the ar tag.
                ArID2_CornerLT = new int[]{0, 0},
                ArID3_CornerLB = new int[]{0, 0},
                ArID4_CornerRB = new int[]{0, 0};
        int ID_1 = 1, ID_2 = 2, ID_3 = 3, ID_4 = 4;

        /* Homography variables */
        final double milToMeter = 0.001;
        final int virtualWidth  = 275,  // Virtual width of AR tag. 	//	Reference ratio from the real tag
                  virtualHeight = 133;	 // Virtual height of AR tag.	//	in millimeter unit.
        final int enlargeRatio = 4;     // Final image size
        final int offsetX = Math.abs((virtualWidth / 2) - (virtualWidth * enlargeRatio / 2)), // Origin shifting for warp process.
                  offsetY = Math.abs((virtualHeight / 2) - (virtualHeight * enlargeRatio / 2));
        MatOfPoint2f corners1 = new MatOfPoint2f(),	//	Use contain input corners of the image A.
                     corners2 = new MatOfPoint2f();	//	Use contain output corners of the image B.
        Mat H;	// Coefficient of Homography.

        /* Get AR tag routine */
        while(!ARFinished && loopCount < loopCountMax)
        {
            if(targetNum == 1)
            {

                for(int i=0; i<2; i++) { moveTo(Pt1, Qua1); }
                if(firstTime)
                {
                    firstTime = false;
                    api.reportPoint1Arrival();
                    api.laserControl(true);
                }
                matSrc = api.getMatNavCam();
            }
            else if(targetNum == 2)
            {
                for(int i=0; i<2; i++) { moveTo(Pt2, Qua2); }
                if(firstTime)
                {
                    firstTime = false;
                    api.laserControl(true);
                }
                matSrc = api.getMatNavCam();
            }

            try
            {
                Aruco.detectMarkers(matSrc, Dict, Corners, IDs); 	// AR tag detector
                AR_ID = new int[] 									// Put ID of AR Code to Array of int.
                {
                    (int) IDs.get(0, 0)[0],
                    (int) IDs.get(1, 0)[0],
                    (int) IDs.get(2, 0)[0],
                    (int) IDs.get(3, 0)[0]
                };
            }
            catch (Exception error)
            {
                // Blank space
            }
            finally
            {
                if (AR_ID[0] != 0 && AR_ID[1] != 0 && AR_ID[2] != 0 && AR_ID[3] != 0)
                { /* The condition is four AR Code reading successful. */

                    if(targetNum == 1)
                    {
                        ID_1 = 1;
                        ID_2 = 2;
                        ID_3 = 3;
                        ID_4 = 4;
                    }
                    else if(targetNum == 2)
                    {
                        ID_1 = 11;
                        ID_2 = 12;
                        ID_3 = 13;
                        ID_4 = 14;
                    }

                    for (int i = 0; i < 4; i++)
                    {
                        double[][] Corner = // Put each AR tag corners to 2d-array of double.
                        {   // Return variables sorted clockwise starting from left-top corner.
                            {(int) Corners.get(i).get(0, 0)[0], (int) Corners.get(i).get(0, 0)[1]},	// Left-top
                            {(int) Corners.get(i).get(0, 2)[0], (int) Corners.get(i).get(0, 2)[1]},	// Right-bottom
                            {(int) Corners.get(i).get(0, 1)[0], (int) Corners.get(i).get(0, 1)[1]},	// Right-top
                            {(int) Corners.get(i).get(0, 3)[0], (int) Corners.get(i).get(0, 3)[1]}	// Left-bottom
                        };
                        /* Set the corner to match the id of ar code. */
                             if (AR_ID[i] == ID_1){ ArID1_CornerRT[0] = (int) Corner[2][0]; ArID1_CornerRT[1] = (int) Corner[2][1]; }
                        else if (AR_ID[i] == ID_2){ ArID2_CornerLT[0] = (int) Corner[0][0]; ArID2_CornerLT[1] = (int) Corner[0][1]; }
                        else if (AR_ID[i] == ID_3){ ArID3_CornerLB[0] = (int) Corner[3][0]; ArID3_CornerLB[1] = (int) Corner[3][1]; }
                        else if (AR_ID[i] == ID_4){ ArID4_CornerRB[0] = (int) Corner[1][0]; ArID4_CornerRB[1] = (int) Corner[1][1]; }
                    }

                    // >>>>>>>>>>>>>>>>> log image
                    if(targetNum == 1)
                    {
                        api.saveMatImage(matSrc, "SourceImage_1.jpg");
                    }
                    else if(targetNum == 2)
                    {
                        api.saveMatImage(matSrc, "SourceImage_2.jpg");
                    }

                    /* homography calculate */
                    org.opencv.core.Point[] cornerIN =  // Set input corners with four AR tag.
                    {
                        new org.opencv.core.Point(ArID1_CornerRT[0], ArID1_CornerRT[1]),
                        new org.opencv.core.Point(ArID2_CornerLT[0], ArID2_CornerLT[1]),
                        new org.opencv.core.Point(ArID3_CornerLB[0], ArID3_CornerLB[1]),
                        new org.opencv.core.Point(ArID4_CornerRB[0], ArID4_CornerRB[1])
                    };
                    org.opencv.core.Point[] cornerOut = // Set output corners with actual ratio of AR tag.
                    {
                        new org.opencv.core.Point(virtualWidth + offsetX, offsetY),
                        new org.opencv.core.Point(offsetX, offsetY),
                        new org.opencv.core.Point(offsetX,virtualHeight + offsetY),
                        new org.opencv.core.Point(virtualWidth + offsetX,virtualHeight + offsetY)
                    };
                    corners1.fromArray(cornerIN);	// input array into matrix of point variable.
                    corners2.fromArray(cornerOut);	// ..
                    H = Calib3d.findHomography(corners1, corners2);	// Calculate constant variables of Homography.
                    Mat homoImg = new Mat(1280, 960, CvType.CV_8UC1);
                    Imgproc.warpPerspective(matSrc, homoImg, H, new Size(virtualWidth * enlargeRatio, virtualHeight * enlargeRatio));

                    // >>>>>>>>>>>>>>>>> log image
                    if(targetNum == 1)
                    {
                        api.saveMatImage(homoImg, "HomographyImage_1.jpg");
                    }
                    else if(targetNum == 2)
                    {
                        api.saveMatImage(homoImg, "HomographyImage_2.jpg");
                    }

                    /* target point calculate */
                    double[] laserCenterSrc = {718, 468};   // Center of laser pointer on image. (Constant point)
                    // old laser point (786, 462)
                    // new 1 (786, 460) // fail
                    // new 2 (729, 436) // close
                    // new 3 (727, 437) // close
                    // new 4 (725, 437) // perfect
                    // new 5 (718, 442) // perfect
                    // new 6 (718, 468) // perfect
                    double[] TargetCenterHomo = {offsetX + (virtualWidth / 2), offsetY + (virtualHeight / 2)};  // Center of Target tag on homo image. (Constant point)
                    double[] laserCenterHomo = SrcPointTransformToHomoPoint(H, laserCenterSrc);                 // Get laser position on homography image.
                    double X_dif = (laserCenterHomo[0] - TargetCenterHomo[0]) * milToMeter;   // The x-axis distance between the robot and the target : meter unit
                    double Y_dif = (laserCenterHomo[1] - TargetCenterHomo[1]) * milToMeter;   // The y-axis distance between the robot and the target : meter unit

                    if(targetNum == 1)
                    {
                        /**/
                        Point newPt1 = new Point(Pt1.getX() + Y_dif, Pt1.getY() - X_dif, Pt1.getZ());
                        for(int i=0; i<2; i++) { moveTo(newPt1, Qua1); }

                        // >>>>>>>>>>>>>>>>> log image
                        api.saveMatImage(api.getMatNavCam(), X_dif + ", " + Y_dif + ".jpg");

                        api.takeTarget1Snapshot();
                        api.laserControl(false);
                    }
                    else if(targetNum == 2)
                    {
                        /**/
                        Point newPt2 = new Point(Pt2.getX() - 0.106, Pt2.getY(), Pt2.getZ() + 0.09);
                        Pt_Robot = Pt2;
                        Pt_Target = new Point(Pt2.getX() + X_dif, -11.500, Pt2.getZ() - Y_dif);
                        // start y value = -10.585
                        // -10.485 high gap
                        // -10.885 close
                        getQuafromVector();

                        /* circle target optimize */
                        double xAvg = 0, yAvg = 0, rAvg = 0;
                        try
                        {
                            Mat circles = new Mat();
                            Imgproc.medianBlur(homoImg, homoImg, 5);
                            Imgproc.HoughCircles(homoImg, circles, Imgproc.HOUGH_GRADIENT, 1, 5, 500, 100, 40, 70);
                            for (int i = 0; i < circles.cols(); i++)
                            {
                                double[] data = circles.get(0, i);
                                xAvg += Math.round(data[0]);
                                yAvg += Math.round(data[1]);
                                rAvg += Math.round(data[2]);
                            }

                            /* new target point calculate */
                            final double shiftXratio = 1;
                            final double shiftYratio = 0.618;
                            double originCircleX = offsetX + (virtualWidth / 2);
                            double originCircleY = offsetY + (virtualHeight / 2);
                            double X_dif2 = (xAvg - originCircleX) * milToMeter;   // The x-axis distance between the robot and the target : meter unit
                            double Y_dif2 = (yAvg - originCircleY) * milToMeter;   // The y-axis distance between the robot and the target : meter unit
                            newPt2 = new Point(newPt2.getX() + (X_dif2 * shiftXratio), newPt2.getY(), newPt2.getZ() + (Y_dif2 * shiftYratio));

                            // >>>>>>>>>>>>>>>>> log image
                            Imgproc.circle(homoImg, new org.opencv.core.Point(xAvg, yAvg), 60, new Scalar(0, 0, 255), 3);
                            api.saveMatImage(homoImg, "HoughCirclesImage.jpg");

                            // >>>>>>>>>>>>>>>>> log image
                            api.saveMatImage(api.getMatNavCam(), X_dif2 + ", " + Y_dif2 + ".jpg");

                        }
                        catch (Exception error)
                        {
                            // Blank space
                        }

                        for(int i=0; i<2; i++) { moveTo(newPt2, Qua_Target); }

                        // >>>>>>>>>>>>>>>>> log image
                        api.saveMatImage(api.getMatNavCam(), X_dif + ", " + Y_dif + ".jpg");

                        api.takeTarget2Snapshot();
                        api.laserControl(false);
                    }
                    ARFinished = true; // Set new condition.
                }
            }
            loopCount++;
        }
        if (!ARFinished)
        {
            if(targetNum == 1)
            {
                api.takeTarget1Snapshot();
            }
            else if(targetNum == 2)
            {
                api.takeTarget2Snapshot();
            }
            api.laserControl(false);
        }
        ARFinished = false;
    }
}