using DigitalAssembly.Photogrammetry.Calibration;
using DigitalAssembly.Photogrammetry.Camera;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using DigitalAssembly.Photogrammetry.Stereo.Geometry;

namespace DigitalAssembly.Photogrammetry.Stereo;

// TODO: Naming refactoring
/// <summary>
/// Top level class for real coordinates computation on camera pair.
/// </summary>
public sealed class StereoSystemGeometry
{
    private readonly StereoGeometry _stereoSystem;
    private readonly EpipolarGeometry _epipolarGeometry;
    private readonly ModelCoordinatesComputation _modelCoordinates;

    private StereoSystemGeometry(StereoGeometry stereo, ModelCoordinatesComputation modelMath, EpipolarGeometry epipolar)
    {
        _stereoSystem = stereo;
        _epipolarGeometry = epipolar;
        _modelCoordinates = modelMath;
    }

    public static StereoSystemGeometry FromCameraModels(CameraModel left, CameraModel right, StereoGeometryParameters stereoGeometryParameters, double myu = 1)
    {
        StereoGeometry st = new(left, right, myu);
        EpipolarGeometry ep = EpipolarGeometry.Init(st, stereoGeometryParameters);
        ModelCoordinatesComputation mo = new(st);
        return new StereoSystemGeometry(st, mo, ep);
    }

    internal static StereoSystemGeometry FromCameraModels(CameraModel left, CameraModel right, EpipolarGeometry epipolarGeometry)
    {
        epipolarGeometry.GlobalParameters.EpipolarEpsilon = 0.0958;
        StereoGeometry st = new(left, right);
        EpipolarGeometry ep = EpipolarGeometry.Init(st, epipolarGeometry.GlobalParameters);
        ModelCoordinatesComputation mo = new(st);
        return new StereoSystemGeometry(st, mo, ep);
    }

    public static StereoSystemGeometry FromProject(Project project) =>
        FromCameraModels(project.LeftCameraModel, project.RightCameraModel, project.ProjectStereoGeometryParameters, project.Myu);

    // TODO: Naming refactoring
    public IEnumerable<MarkPoint<ModelCsPoint>> Compute3DCoords(List<MarkPoint<PixelCsPoint>> picturePointsLeft, List<MarkPoint<PixelCsPoint>> picturePointsRight, bool scaleBar = false)
    {
        picturePointsLeft.ForEach(p => p.MarkCode.Code = -1);
        picturePointsRight.ForEach(p => p.MarkCode.Code = -1);

        (IEnumerable<MarkPoint<UndistortedPictureCsPoint>> leftPoints, IEnumerable<MarkPoint<UndistortedPictureCsPoint>> rightPoints) =
            _stereoSystem.Undistort(picturePointsLeft, picturePointsRight);

        IEnumerable<MarkPointPair<HomogeneousPictureCsPoint>> pairs =
            _epipolarGeometry.PairMarks(leftPoints, rightPoints, scaleBar);

        IEnumerable<MarkPointPair<CameraCsPoint>> pairsInCameraCoords = _stereoSystem.ConvertPairsToCameraCoordinates(pairs);

        // Computing 3D coordinates
        foreach (MarkPointPair<CameraCsPoint> pair in pairsInCameraCoords)
        {
            yield return _modelCoordinates.Compute3DPoint(pair);
        }
    }

    public IEnumerable<MarkPoint<ModelCsPoint>> Compute3DCoords2(List<MarkPoint<PixelCsPoint>> picturePointsLeft, List<MarkPoint<PixelCsPoint>> picturePointsRight)
    {
        (IEnumerable<MarkPoint<UndistortedPictureCsPoint>> leftPoints, IEnumerable<MarkPoint<UndistortedPictureCsPoint>> rightPoints) =
             _stereoSystem.Undistort(picturePointsLeft, picturePointsRight);
        picturePointsLeft.ForEach(p => p.MarkCode.Code = -1);
        picturePointsRight.ForEach(p => p.MarkCode.Code = -1);

        IEnumerable<MarkPointPair<HomogeneousPictureCsPoint>> pairs =
            _epipolarGeometry.PairMarks(leftPoints, rightPoints);

        IEnumerable<MarkPointPair<CameraCsPoint>> pairsInCameraCoords = _stereoSystem.ConvertPairsToCameraCoordinates(pairs);

        // Computing 3D coordinates
        foreach (MarkPointPair<CameraCsPoint> pair in pairsInCameraCoords)
        {
            yield return _modelCoordinates.Compute3DPoint2(pair);
        }
    }

    public StereoSystemGeometry Calibrate(CalibrationMethod calibrationMethod)
    {
        double[][] result = _stereoSystem.Calibrate(calibrationMethod);

        return this;
        //if (result[1][0] is 0)
        //{
        //    return this;
        //}

        //Rotation newLeftRot = new(new(Angle.FromRadians(result[0][0]), Angle.FromRadians(result[0][1]), Angle.FromRadians(result[0][2])), Math.Common.Enums.EulerAngleConvention.ZXY);
        //ModelCsPoint newLeftTranslation = new(result[1][0], result[1][1], result[1][2]);
        //Transformation3D<ModelCsPoint> newLeftExtrincic = new(newLeftRot, newLeftTranslation);

        //CameraModel newLeft = _stereoSystem.Left.CameraModel;

        //Rotation newRightRot = new(new(Angle.FromRadians(result[2][0]), Angle.FromRadians(result[2][1]), Angle.FromRadians(result[2][2])), Math.Common.Enums.EulerAngleConvention.XYZ);
        //ModelCsPoint newRightTranslation = new(-result[3][0], -result[3][1], -result[3][2]);
        //Transformation3D<ModelCsPoint> newRightExtrincic = new(newRightRot, newRightTranslation);

        //CameraModel newRight = new CameraModel(newRightExtrincic,
        //                                      _stereoSystem.Right.CameraModel.IntrisicParameters,
        //                                      _stereoSystem.Right.CameraModel.ImageCentre,
        //                                      _stereoSystem.Right.CameraModel.ScaleParameter,
        //                                      _stereoSystem.Right.CameraModel.ImageSize);

        ////TODO попробовать вариант расчета с не нулевой матрицей для левой камеры и принимать в качестве результата калибровки 4 матрицы а не две перехода в левую 
        //return FromCameraModels(newLeft, newRight, _epipolarGeometry);
    }

}