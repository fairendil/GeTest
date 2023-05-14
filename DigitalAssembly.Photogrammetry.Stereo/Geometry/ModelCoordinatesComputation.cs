using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Photogrammetry.Stereo.Geometry;

internal class ModelCoordinatesComputation : ICoordinateComputation
{
    private readonly Vector<double> _MainAxis;
    private readonly double _Myu;
    private readonly Matrix<double> _RotationLeft, _RotationRight;

    public ModelCoordinatesComputation(StereoGeometry stereo)
    {
        _RotationLeft = stereo.Rotation.left;
        _RotationRight = stereo.Rotation.right;
        _MainAxis = stereo.Translation.right * stereo.Myu;
        _Myu = stereo.Myu;
    }

    private Vector<double> Point(Vector<double> left, Vector<double> right)
    {
        Vector<double> l = left;
        Vector<double> r = _RotationRight * right;

        double lambd = ((_MainAxis[0] * r[2]) - (_MainAxis[2] * r[0])) / ((l[0] * r[2]) - (r[0] * l[2]));
        double mu = ((_MainAxis[0] * l[2]) - (_MainAxis[2] * l[0])) / ((l[0] * r[2]) - (r[0] * l[2]));
        double X = 0 + (lambd * l[0]);
        // double X1 = MainAxis[0] + (mu * r[0]) equals X
        double Y1 = 0 + (lambd * l[1]), Y2 = _MainAxis[1] + (mu * r[1]);
        double Z = 0 + (lambd * l[2]);
        // double Z1 = MainAxis[2] + (mu * r[2]) equals Z
        double parallax = Y2 - Y1;
        Console.WriteLine($"Parallax: {parallax}");
        double Y = (Y1 + Y2) / 2;

        return Vector<double>.Build.DenseOfArray(new double[3] { X, Y, Z });
    }

    private Vector<double> Point2(Vector<double> left, Vector<double> right)
    {
        Vector<double> l = left;
        Vector<double> r = _RotationRight * right;
        Matrix<double> mat = Matrix<double>.Build.Dense(3, 2);
        mat[0, 0] = l[0];
        mat[0, 1] = r[0];
        mat[1, 0] = l[1];
        mat[1, 1] = r[1];
        mat[2, 0] = l[2];
        mat[2, 1] = r[2];
        Vector<double> res = mat.Solve(_MainAxis);
        return Vector<double>.Build.DenseOfArray(new double[] { res[0] * l[0], res[0] * l[1], res[0] * l[2] });
    }

    /// <summary>
    /// Computes 3D coordinate as center of line segment between skew lines
    /// </summary>
    /// <param name="pair">Pair of CameraCsPoints to solve</param>
    /// <returns></returns>
    public MarkPoint<ModelCsPoint> Compute3DPoint(MarkPointPair<CameraCsPoint> pair)
    {
        Vector<double> coords = Point(pair.LeftPoint.Coordinate, pair.RightPoint.Coordinate);
        return new(pair.MarkCode, new(coords[0], coords[1], coords[2]));
    }

    /// <summary>
    /// Computes 3D coordinate bt solving system of linear equations
    /// </summary>
    /// <param name="pair">Pair of CameraCsPoints to solve</param>
    /// <returns></returns>
    public MarkPoint<ModelCsPoint> Compute3DPoint2(MarkPointPair<CameraCsPoint> pair)
    {
        Vector<double> coords = Point2(pair.LeftPoint.Coordinate, pair.RightPoint.Coordinate);
        return new(pair.MarkCode, new(coords[0], coords[1], coords[2]));
    }

    public MarkPoint<CameraCsPoint> Compute2DPoint(MarkPoint<CameraCsPoint> pair, MarkPoint<ModelCsPoint> modelCsPoint)
    {
        Vector<Double> coords = Point2D(pair.Point.Coordinate, modelCsPoint.Point.Coordinate);
        return new(pair.MarkCode, new(coords[0], coords[1], coords[2]));
    }

    private Vector<double> Point2D(Vector<double> pair, Vector<double> model)
    {
        Vector<double> l = pair;
        Vector<double> r = _RotationRight * model;

        double lambd = ((_MainAxis[0] * r[2]) - (_MainAxis[2] * r[0])) / ((l[0] * r[2]) - (r[0] * l[2]));
        double mu = ((_MainAxis[0] * l[2]) - (_MainAxis[2] * l[0])) / ((l[0] * r[2]) - (r[0] * l[2]));
        double X = 0 + (lambd * l[0]);
        // double X1 = MainAxis[0] + (mu * r[0]) equals X
        double Y1 = 0 + (lambd * l[1]), Y2 = _MainAxis[1] + (mu * r[1]);
        double Z = 0 + (lambd * l[2]);
        // double Z1 = MainAxis[2] + (mu * r[2]) equals Z
        double parallax = Y2 - Y1;
        Console.WriteLine($"Parallax: {parallax}");
        double Y = (Y1 + Y2) / 2;
        return Vector<double>.Build.DenseOfArray(new double[3] { X, Y, Z });
    }
}
