using DigitalAssembly.Math.Matrices;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using MathNet.Spatial.Euclidean;
using System.Drawing;

namespace DigitalAssembly.Photogrammetry.Calibration;

public class CalibrationMethod
{
    private const int V = 1;
    private MCvPoint3D32f[][] _objectPoints = new MCvPoint3D32f[V][];
    private PointF[][] _imagePointsLeft = new PointF[V][];
    private PointF[][] _imagePointsRight = new PointF[V][];
    private bool _drawLastImage = false;
    private string _imageName;
    private int _index;
    private Matrix<double> _cameraMatrixLeft;
    private Matrix<double> _distCoeffsLeft;
    private Matrix<double> _cameraMatrixRight;
    private Matrix<double> _distCoeffsRight;

    public CalibrationMethod(IEnumerable<MarkPoint<ModelCsPoint>> markPoints,
                             IEnumerable<MarkPoint<PixelCsPoint>> leftPoits,
                             IEnumerable<MarkPoint<PixelCsPoint>> rightPoints,
                             string imageName,
                             int index)
    {
        int i = 0;// _objectPoints.Count(o => o != null);
        _objectPoints[i] = markPoints.Select(p => new MCvPoint3D32f((float)p.Point.X, (float)p.Point.Y, (float)p.Point.Z)).ToArray();
        _imagePointsLeft[i] = leftPoits.Select(p => new PointF((float)p.Point.X, (float)p.Point.Y)).ToArray();
        _imagePointsRight[i] = rightPoints.Select(p => new PointF((float)p.Point.X, (float)p.Point.Y)).ToArray();
        _imageName = imageName;
        _index = index;
    }

    // Summary:
    //     Estimates transformation between the 2 cameras making a stereo pair. If we have
    //     a stereo camera, where the relative position and orientatation of the 2 cameras
    //     is fixed, and if we computed poses of an object relative to the fist camera and
    //     to the second camera, (R1, T1) and (R2, T2), respectively (that can be done with
    //     cvFindExtrinsicCameraParams2), obviously, those poses will relate to each other,
    //     i.e. given (R1, T1) it should be possible to compute (R2, T2) - we only need
    //     to know the position and orientation of the 2nd camera relative to the 1st camera.
    //     That's what the described function does. It computes (R, T) such that: R2=R*R1,
    //     T2=R*T1 + T
    public double[][] Calibrate(MathNet.Numerics.LinearAlgebra.Matrix<double> intrisicMatrixLeft,
                                 MathNet.Numerics.LinearAlgebra.Matrix<double> intrisicMatrixRight,
                                 MathNet.Numerics.LinearAlgebra.Matrix<double> distortionCoeffsLeft,
                                 MathNet.Numerics.LinearAlgebra.Matrix<double> distortionCoeffsRight,
                                 MathNet.Numerics.LinearAlgebra.Matrix<double> rotationLeft,
                                 MathNet.Numerics.LinearAlgebra.Matrix<double> translationLeft,
                                 MathNet.Numerics.LinearAlgebra.Matrix<double> rotationRight,
                                 MathNet.Numerics.LinearAlgebra.Matrix<double> translationRight,
                                 Size imageSize,
                                 double reprojectionErrorTreshold)
    {
        double[][] result = new double[7][];
        for (int i = 0; i < result.Length; i++)
        {
            switch (i)
            {
                case 0:
                case 2:
                case 4:
                    result[i] = new double[9];
                    break;
                case 6:
                    result[i] = new[] { reprojectionErrorTreshold };
                    break;
                default:
                    result[i] = new double[3];
                    break;
            }
        }

        _cameraMatrixLeft ??= new(intrisicMatrixLeft.ToArray());
        _distCoeffsLeft ??= new(distortionCoeffsLeft.ToArray()); //new Matrix<double>(1, 14);

        _cameraMatrixRight ??= new(intrisicMatrixRight.ToArray());
        _distCoeffsRight ??= new(distortionCoeffsRight.ToArray()); //new Matrix<double>(1, 14);

        Matrix<double> rl = new Matrix<double>(rotationLeft.ToArray());
        using Mat rotLeft = rl.Mat;

        var tl = new Matrix<double>(translationLeft.ToArray());
        using Mat transLeft = tl.Mat;

        var rr = new Matrix<double>(rotationRight.ToArray());
        using Mat rotRight = rr.Mat;

        var tr = new Matrix<double>(translationRight.ToArray());
        using Mat transRight = tr.Mat;

        MCvTermCriteria termCrit = new MCvTermCriteria(0.1e-5);

        using Mat rotationCameraLeft = new Mat();
        using Mat translationCameraLeft = new Mat();
        using Mat rotationCameraRight = new Mat();
        using Mat translationCameraRight = new Mat();
        using Mat stereoRotation = new Mat();
        using Mat stereoTraslation = new Mat();
        using Mat fundamtal = new Mat();
        using Mat essential = new Mat();

        MCvPoint3D32f[][] obj = new MCvPoint3D32f[1][];

        //termCrit = new MCvTermCriteria(0.1e-4);

        rotLeft.GetOutputArray().CopyTo(rotationCameraLeft);
        transLeft.GetOutputArray().CopyTo(translationCameraLeft);
        rotRight.GetOutputArray().CopyTo(rotationCameraRight);
        transRight.GetOutputArray().CopyTo(translationCameraRight);
        rotationCameraLeft.CopyTo(result[0]);
        translationCameraLeft.CopyTo(result[1]);
        rotationCameraRight.CopyTo(result[4]);
        translationCameraRight.CopyTo(result[5]);

        CalibrateCamera(imageSize, _cameraMatrixLeft, _distCoeffsLeft, termCrit, rotationCameraLeft, translationCameraLeft, _objectPoints[0], _imagePointsLeft[0], true);
        CalibrateCamera(imageSize, _cameraMatrixRight, _distCoeffsRight, termCrit, rotationCameraRight, translationCameraRight, _objectPoints[0], _imagePointsRight[0], true);

        bool solvedPnPForCameraLeft = CvInvoke.SolvePnP(_objectPoints[0], _imagePointsLeft[0], _cameraMatrixLeft, _distCoeffsLeft, rotationCameraLeft, translationCameraLeft, true, SolvePnpMethod.EPnP);
        bool solvedPnPForCameraRight = CvInvoke.SolvePnP(_objectPoints[0], _imagePointsRight[0], _cameraMatrixRight, _distCoeffsRight, rotationCameraRight, translationCameraRight, true, SolvePnpMethod.EPnP);

        double reprojectionErrorLeft = CalculatePointReprojectionError(_objectPoints[0], _imagePointsLeft[0], _cameraMatrixLeft, _distCoeffsLeft, rotationCameraLeft, translationCameraLeft);
        double reprojectionErrorRight = CalculatePointReprojectionError(_objectPoints[0], _imagePointsRight[0], _cameraMatrixRight, _distCoeffsRight, rotationCameraRight, translationCameraRight);

        double[] translationRightPnP = new double[3];
        translationCameraRight.CopyTo(translationRightPnP);

        StreamWriter translationFileLeft = File.AppendText("CalibrationData//translationRightPnP.txt");

        translationFileLeft.WriteLine($"{translationRightPnP[0]} {translationRightPnP[1]} {translationRightPnP[2]} {reprojectionErrorRight}");
        translationFileLeft.Dispose();

        double stereoReprojectionError = (reprojectionErrorLeft + reprojectionErrorRight) / 2;
        //преобразуем оба поворота в матричный вид            
        CvInvoke.Rodrigues(rotationCameraLeft, rotationCameraLeft);
        CvInvoke.Rodrigues(rotationCameraRight, rotationCameraRight);

        if (stereoReprojectionError <= reprojectionErrorTreshold && solvedPnPForCameraLeft && solvedPnPForCameraRight)
        {
            //Расчет матриц RT взаимного ориентирования
            //H. Zhao2015 10
            Mat invertCameraLeftRotationMatrix = new Mat();
            CvInvoke.Invert(rotationCameraLeft, invertCameraLeftRotationMatrix, DecompMethod.Svd);

            CvInvoke.Multiply(rotationCameraRight, invertCameraLeftRotationMatrix, stereoRotation);

            Mat translationCameraLeftRotated = new Mat();
            CvInvoke.Rodrigues(stereoRotation, stereoRotation);
            CvInvoke.Multiply(stereoRotation, translationCameraLeft, translationCameraLeftRotated);
            CvInvoke.Rodrigues(stereoRotation, stereoRotation);
            (translationCameraRight - translationCameraLeftRotated).CopyTo(stereoTraslation);
            rotationCameraLeft.CopyTo(result[0]);
            translationCameraLeft.CopyTo(result[1]);
            rotationCameraRight.CopyTo(result[4]);
            translationCameraRight.CopyTo(result[5]);
            result[6][0] = stereoReprojectionError;
        }

        double[] transitionFromMath = new double[3];
        double[] transitionFromOpenCv = new double[3];
        stereoTraslation.CopyTo(transitionFromMath);

        StreamWriter translationFileMath = File.AppendText("CalibrationData//transitionFromMath.txt");

        translationFileMath.WriteLine($"{transitionFromMath[0]} {transitionFromMath[1]} {transitionFromMath[2]} {stereoReprojectionError}");
        translationFileMath.Dispose();

        double reprojectionError = StereoCalibrate(imageSize,
                        _cameraMatrixLeft,
                        _distCoeffsLeft,
                        _cameraMatrixRight,
                        _distCoeffsRight,
                        termCrit,
                        stereoRotation,
                        stereoTraslation,
                        fundamtal,
                        essential);

        stereoTraslation.CopyTo(transitionFromOpenCv);
        StreamWriter transitionFromOpenCV = File.AppendText("CalibrationData//transitionFromOpenCV.txt");

        transitionFromOpenCV.WriteLine($"{transitionFromOpenCv[0]} {transitionFromOpenCv[1]} {transitionFromOpenCv[2]} {reprojectionError}");
        transitionFromOpenCV.Dispose();

        stereoRotation.CopyTo(result[2]);
        stereoTraslation.CopyTo(result[3]);

        return result;
    }

    private MathNet.Numerics.LinearAlgebra.Matrix<double> GetMathNetRotation(Mat rotationCameraLeft)
    {
        MathNet.Numerics.LinearAlgebra.Matrix<double> rotationLeft;
        double[][] buferArray = new double[3][];
        buferArray[0] = new double[3];
        buferArray[1] = new double[3];
        buferArray[2] = new double[3];
        rotationCameraLeft.Col(0).CopyTo(buferArray[0]);
        rotationCameraLeft.Col(1).CopyTo(buferArray[1]);
        rotationCameraLeft.Col(2).CopyTo(buferArray[2]);
        rotationLeft = MathNet.Numerics.LinearAlgebra.Matrix<double>.Build.DenseOfColumnArrays(buferArray);
        return rotationLeft;
    }

    public double CalculatePointReprojectionError(MCvPoint3D32f[] objectPoint, PointF[] imagePoints, Matrix<double> cameraMatrix, Matrix<double> distCoeffs, Mat rotation, Mat translation)
    {
        double error = 0;

        PointF[] projection = CvInvoke.ProjectPoints(objectPoint, rotation, translation, cameraMatrix, distCoeffs);

        for (int index = 0; index < objectPoint.Count(); index++)
        {
            PointF projectionPoin = projection[index];
            PointF imagePoint = imagePoints[index];
            double sX = System.Math.Pow(projectionPoin.X - imagePoint.X, 2);
            double sY = System.Math.Pow(projectionPoin.Y - imagePoint.Y, 2);
            error += System.Math.Sqrt(sX + sY);
        }

        error /= objectPoint.Count();
        return error;
    }

    private double CalibrateCamera(Size imageSize, Matrix<double> cameraMatrix2, Matrix<double> distCoeffs2, MCvTermCriteria termCrit, Mat rotationCamera, Mat translationCamera, MCvPoint3D32f[] objectPoints, PointF[] img2, bool isLeft = false)
    {
        MCvPoint3D32f[][] obj = new MCvPoint3D32f[1][];
        PointF[][] img = new PointF[1][];
        Mat[] rotationVector = new Mat[1];
        rotationVector[0] = new Mat();
        Mat[] translationVector = new Mat[1];
        translationVector[0] = new Mat();
        obj[0] = objectPoints;
        img[0] = img2;

        CalibType calibType = isLeft
            ? CalibType.UseIntrinsicGuess
            //| CalibType.RationalModel
            //| CalibType.ThinPrismModel
            //| CalibType.TiltedModel
            : CalibType.UseIntrinsicGuess
            | CalibType.FixAspectRatio
            | CalibType.FixFocalLength
            | CalibType.RationalModel
            | CalibType.ThinPrismModel
            | CalibType.TiltedModel
            ;

        double reprojectionError = CvInvoke.CalibrateCamera(obj,
                                        img,
                                        imageSize,
                                        cameraMatrix2,
                                        distCoeffs2,
                                        calibType,
                                        termCrit,
                                        out rotationVector,
                                        out translationVector);

        if (reprojectionError < 1)
        {
            rotationVector[0].CopyTo(rotationCamera);
            translationVector[0].CopyTo(translationCamera);
        }

        return reprojectionError;
    }

    private double StereoCalibrate(Size imageSize, Matrix<double> cameraMatrix1, Matrix<double> distCoeffs1, Matrix<double> cameraMatrix2, Matrix<double> distCoeffs2, MCvTermCriteria termCrit, Mat stereoRotation, Mat stereoTrasition, Mat fundamtal, Mat essential, int index = -1)
    {
        if (index == -1)
        {
            return CvInvoke.StereoCalibrate(
            _objectPoints, _imagePointsLeft, _imagePointsRight,
            cameraMatrix1, distCoeffs1,
            cameraMatrix2, distCoeffs2,
            imageSize,
            stereoRotation,
            stereoTrasition,
            essential,
            fundamtal,
            CalibType.UseIntrinsicGuess
            | CalibType.FixFocalLength
            | CalibType.FixPrincipalPoint,
            termCrit);
        }

        MCvPoint3D32f[][] obj = new MCvPoint3D32f[1][];
        obj[0] = _objectPoints[index];
        PointF[][] img1 = new PointF[1][];
        PointF[][] img2 = new PointF[1][];
        img1[0] = _imagePointsLeft[index];
        img2[0] = _imagePointsRight[index];

        return CvInvoke.StereoCalibrate(
            obj, img1, img2,
            cameraMatrix1, distCoeffs1,
            cameraMatrix2, distCoeffs2,
            imageSize,
            stereoRotation,
            stereoTrasition,
            essential,
            fundamtal,
            CalibType.UseIntrinsicGuess
            | CalibType.FixFocalLength
            | CalibType.FixPrincipalPoint,
            termCrit);
    }
}
