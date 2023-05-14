using DigitalAssembly.Photogrammetry.Camera;
using DigitalAssembly.Photogrammetry.Camera.DistortionModels;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using MathNet.Numerics.LinearAlgebra;
using static DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems.CoordinateSystemTransformations;

namespace DigitalAssembly.Photogrammetry.Stereo.Geometry;

internal class CameraGeometry
{
    private readonly CameraModel _cameraModel;
    private readonly IDistortion _distortion;

    public CameraGeometry(CameraModel model)
    {
        _cameraModel = model;
        _distortion = new ClassicDistortion(_cameraModel);
        Translation = _cameraModel.ExtrisicParameters.Point3D.Coordinate;
        Rotation = _cameraModel.ExtrisicParameters.Rotation.Matrix;
        IntrisicMatrix = Matrix<double>.Build.DenseOfArray(new double[,]
                           { { _cameraModel.IntrisicParameters.Focus, 0, _cameraModel.IntrisicParameters.PrincipalPoint.X },
                             { 0, _cameraModel.IntrisicParameters.Focus, _cameraModel.IntrisicParameters.PrincipalPoint.Y },
                             { 0, 0, 1 } });
    }

    public Vector<double> Translation { get; }

    public Matrix<double> Rotation { get; }

    public Matrix<double> IntrisicMatrix { get; }

    public CameraModel CameraModel => _cameraModel;

    public IEnumerable<MarkPoint<UndistortedPictureCsPoint>> Undistort(IEnumerable<MarkPoint<PixelCsPoint>> initial)
    {
        // Converting points into picture plane coordinates
        IEnumerable<MarkPoint<PictureCsPoint>> pointsPicture = ConvertToPictureCs(initial);

        // Undistortion
        return _distortion.Undistort(pointsPicture);
    }

    private IEnumerable<MarkPoint<PictureCsPoint>> ConvertToPictureCs(IEnumerable<MarkPoint<PixelCsPoint>> initial)
    {
        foreach (MarkPoint<PixelCsPoint> point in initial)
        {
            yield return point.UpdateCoordinate(point.Point.ToPictureCsPoint(CameraModel.ImageCentre, CameraModel.ScaleParameter));
        }
    }

    internal IEnumerable<MarkPoint<CameraCsPoint>> ConvertToCameraCs(IEnumerable<MarkPoint<PictureCsPoint>> points)
    {
        foreach (MarkPoint<PictureCsPoint> point in points)
        {
            yield return point.UpdateCoordinate(point.Point.ToCameraCsPoint(CameraModel.IntrisicParameters.PrincipalPoint, CameraModel.IntrisicParameters.Focus));
        }
    }

    internal MarkPoint<CameraCsPoint> ConvertPointToCameraCs(MarkPoint<HomogeneousPictureCsPoint> point)
     => point.UpdateCoordinate(point.Point.ToCameraCsPoint(CameraModel.IntrisicParameters.PrincipalPoint, CameraModel.IntrisicParameters.Focus));

    internal CameraCsPoint ConvertPointToCameraCs(HomogeneousPictureCsPoint point)
     => point.ToCameraCsPoint(CameraModel.IntrisicParameters.PrincipalPoint, CameraModel.IntrisicParameters.Focus);

}
