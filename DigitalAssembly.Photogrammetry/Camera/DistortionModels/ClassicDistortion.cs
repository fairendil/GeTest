using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using static System.Math;
using System.Collections.Concurrent;

namespace DigitalAssembly.Photogrammetry.Camera.DistortionModels;

public sealed class ClassicDistortion : IDistortion
{
    private readonly double _balancedRadialParam;
    private readonly ClassicDistortionParameters _classicDistortionParameters;
    private readonly PictureCsPoint _principalPoint;
    public ClassicDistortion(CameraModel camera)
    {
        _classicDistortionParameters = camera.IntrisicParameters.ClassicDistortionParameters;
        _principalPoint = camera.IntrisicParameters.PrincipalPoint;

        (double widthHalf, double heightHalf) = ((camera.ImageCentre.X * camera.ScaleParameter.X) + camera.IntrisicParameters.PrincipalPoint.X,
                                                 (camera.ImageCentre.Y * camera.ScaleParameter.Y) + camera.IntrisicParameters.PrincipalPoint.Y);
        double r0 = 2 * Sqrt(Pow(widthHalf, 2) + Pow(heightHalf, 2)) / 3;
        double r0_2 = r0 * r0;
        _balancedRadialParam = (_classicDistortionParameters.A1 * r0_2) +
                               (_classicDistortionParameters.A2 * r0_2 * r0_2) +
                               (_classicDistortionParameters.A3 * r0_2 * r0_2 * r0_2);
    }

    private double RadialDelta(double distance)
    {
        double distance_3 = distance * distance * distance;
        return (_classicDistortionParameters.A1 * distance_3) +
               (_classicDistortionParameters.A2 * distance_3 * (distance * distance)) +
               (_classicDistortionParameters.A3 * distance_3 * distance_3 * distance) - (distance * _balancedRadialParam);
    }

    private PictureCsPoint Radial(PictureCsPoint point, double distance) => point * RadialDelta(distance) / distance;

    private PictureCsPoint Tangential(PictureCsPoint point, double distance)
    {
        double distance_2 = distance * distance;
        return new PictureCsPoint(
            (_classicDistortionParameters.B1 * (distance_2 + (2 * point.X * point.X))) + (_classicDistortionParameters.B2 * 2 * point.X * point.Y),
            (_classicDistortionParameters.B2 * (distance_2 + (2 * point.Y * point.Y))) + (_classicDistortionParameters.B1 * 2 * point.X * point.Y)
        );
    }

    private PictureCsPoint Affine(PictureCsPoint point)
    {
        return new PictureCsPoint(

            (_classicDistortionParameters.C1 * point.X) + (_classicDistortionParameters.C2 * point.Y),
            0
        );
    }

    public IEnumerable<MarkPoint<UndistortedPictureCsPoint>> Undistort(IEnumerable<MarkPoint<PictureCsPoint>> points)
    {
        // TODO: Add concurrent if no more nessesary provide order of points
        //ConcurrentBag<MarkPoint<UndistortedPictureCsPoint>> result = new()
        List<MarkPoint<UndistortedPictureCsPoint>> result = new();

        foreach (MarkPoint<PictureCsPoint> point in points)
        //Parallel.ForEach(points, point =>
        {
            PictureCsPoint vec = point.Point - _principalPoint;
            double distance = vec.L2Norm();
            PictureCsPoint rad = Radial(vec, distance);
            PictureCsPoint tan = Tangential(vec, distance);
            PictureCsPoint aff = Affine(vec);
            result.Add(point.UpdateCoordinate(point.Point.ToUndistortedPoint(rad + tan + aff)));
            //})
        }

        return result;
    }
}
