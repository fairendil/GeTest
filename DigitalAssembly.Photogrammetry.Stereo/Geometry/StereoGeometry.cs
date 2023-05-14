using DigitalAssembly.Photogrammetry.Calibration;
using DigitalAssembly.Photogrammetry.Camera;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Photogrammetry.Stereo.Geometry;

internal class StereoGeometry
{
    private readonly CameraGeometry _Left, _Right;
    private readonly (double Left, double Right) _ImageArea;
    private double[][] _result;

    public CameraGeometry Left => _Left;
    public CameraGeometry Right => _Right;

    public (double Left, double Right) ImageArea => _ImageArea;

    public double Myu { get; }

    public StereoGeometry(CameraModel left, CameraModel right, double myu = 1)
    {
        _Left = new(left);
        _Right = new(right);
        _ImageArea = (4 * left.ImageCentre.X * left.ImageCentre.Y * left.ScaleParameter.X * left.ScaleParameter.Y, 4 * right.ImageCentre.X * right.ImageCentre.Y * right.ScaleParameter.X * right.ScaleParameter.Y);
        Myu = myu;
    }

    public (Vector<double> left, Vector<double> right) Translation => (_Left.Translation, _Right.Translation);

    public (Matrix<double> left, Matrix<double> right) Rotation => (_Left.Rotation, _Right.Rotation);

    public (Matrix<double> left, Matrix<double> right) Intrisic => (_Left.IntrisicMatrix, _Right.IntrisicMatrix);

    public (IEnumerable<MarkPoint<UndistortedPictureCsPoint>> left, IEnumerable<MarkPoint<UndistortedPictureCsPoint>> right)
        Undistort(IEnumerable<MarkPoint<PixelCsPoint>> left, IEnumerable<MarkPoint<PixelCsPoint>> right)
    {
        IEnumerable<MarkPoint<UndistortedPictureCsPoint>> leftPoints = _Left.Undistort(left);
        IEnumerable<MarkPoint<UndistortedPictureCsPoint>> rightPoints = _Right.Undistort(right);
        return (leftPoints, rightPoints);
    }

    public IEnumerable<MarkPointPair<CameraCsPoint>> ConvertPairsToCameraCoordinates(IEnumerable<MarkPointPair<HomogeneousPictureCsPoint>> pairs)
    {
        foreach (MarkPointPair<HomogeneousPictureCsPoint> pair in pairs)
        {
            yield return new(pair.MarkCode, _Left.ConvertPointToCameraCs(pair.LeftPoint), _Right.ConvertPointToCameraCs(pair.RightPoint));
        }
    }

    public double[][] Calibrate(CalibrationMethod calibrationMethod)
    {
        var leftCameraMatrix = Matrix<double>.Build.DenseOfArray(new double[,]
                           { { -Left.CameraModel.IntrisicParameters.Focus/Left.CameraModel.ScaleParameter.X, 0, Left.CameraModel.ImageCentre.X + Left.CameraModel.IntrisicParameters.PrincipalPoint.X/Left.CameraModel.ScaleParameter.X },
                             { 0, -Left.CameraModel.IntrisicParameters.Focus/Left.CameraModel.ScaleParameter.Y, Left.CameraModel.ImageCentre.Y + Left.CameraModel.IntrisicParameters.PrincipalPoint.Y/Left.CameraModel.ScaleParameter.Y },
                             { 0, 0, 1 } });
        var rightCameraMatrix = Matrix<double>.Build.DenseOfArray(new double[,]
                           { { -Right.CameraModel.IntrisicParameters.Focus/Right.CameraModel.ScaleParameter.X, 0, Right.CameraModel.ImageCentre.X + (Right.CameraModel.IntrisicParameters.PrincipalPoint.X/Right.CameraModel.ScaleParameter.X) },
                             { 0, -Right.CameraModel.IntrisicParameters.Focus/Right.CameraModel.ScaleParameter.Y, Right.CameraModel.ImageCentre.Y + (Right.CameraModel.IntrisicParameters.PrincipalPoint.Y/Right.CameraModel.ScaleParameter.Y) },
                             { 0, 0, 1 } });
        var leftCameraDistortion = Matrix<double>.Build.DenseOfRowArrays(
            new double[]
            {
                Left.CameraModel.IntrisicParameters.ClassicDistortionParameters.A1,
                Left.CameraModel.IntrisicParameters.ClassicDistortionParameters.A2,
                Left.CameraModel.IntrisicParameters.ClassicDistortionParameters.B1,
                Left.CameraModel.IntrisicParameters.ClassicDistortionParameters.B2,
                Left.CameraModel.IntrisicParameters.ClassicDistortionParameters.A3,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0
            });
        var rightCameraDistortion =
            Matrix<double>.Build.DenseOfRowArrays(
            new double[]
            {
                Right.CameraModel.IntrisicParameters.ClassicDistortionParameters.A1,
                Right.CameraModel.IntrisicParameters.ClassicDistortionParameters.A2,
                Right.CameraModel.IntrisicParameters.ClassicDistortionParameters.B1,
                Right.CameraModel.IntrisicParameters.ClassicDistortionParameters.B2,
                Right.CameraModel.IntrisicParameters.ClassicDistortionParameters.A3,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0
            });
        var rightTranslationCoordinate = Right.CameraModel.ExtrisicParameters.Point3DCV.Coordinate;

        if (_result == null)
        {
            _result = calibrationMethod.Calibrate(leftCameraMatrix,
                                             rightCameraMatrix,
                                             leftCameraDistortion,
                                             rightCameraDistortion,
                                            Matrix<double>.Build.DenseOfArray(new double[,] { { 0, 0, 0 },
                                                                                               { 0, 0, 0 },
                                                                                               { 0, 0, 0 }}),
                                             Matrix<double>.Build.DenseOfArray(new double[,] { { 0, 0, 0 } }),
                                             Matrix<double>.Build.DenseOfArray(new double[,] { { 0, 0, 0 },
                                                                                               { 0, 0, 0 },
                                                                                               { 0, 0, 0 }}),
                                             Matrix<double>.Build.DenseOfArray(new double[,] { { 0, 0, 0 } }),
                                             new System.Drawing.Size((int)Left.CameraModel.ImageSize.Width, (int)Left.CameraModel.ImageSize.Height),
                                             1);

        }
        else
        {
            _result = calibrationMethod.Calibrate(leftCameraMatrix,
                                                 rightCameraMatrix,
                                                 leftCameraDistortion,
                                                 rightCameraDistortion,
                                                 Matrix<double>.Build.DenseOfColumnArrays(_result[0]),
                                                 Matrix<double>.Build.DenseOfColumnArrays(_result[1]),
                                                 Matrix<double>.Build.DenseOfColumnArrays(_result[4]),
                                                 Matrix<double>.Build.DenseOfColumnArrays(_result[5]),
                                                 new System.Drawing.Size((int)Left.CameraModel.ImageSize.Width, (int)Left.CameraModel.ImageSize.Height),
                                                 _result[6][0]);
        }

        return _result;
    }

}
