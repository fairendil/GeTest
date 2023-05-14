using DigitalAssembly.GoldenEye.DB;
using DigitalAssembly.Math.Common;
using DigitalAssembly.Photogrammetry;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using MathNet.Spatial.Units;
using MathNet.Spatial.Euclidean;
using MathNet.Numerics.LinearAlgebra;
using DigitalAssembly.Math.Common.Enums;
using DigitalAssembly.Math.Common.Interfaces;
using DigitalAssembly.Math.PointClouds;

namespace DigitalAssembly.GoldenEye.Objects;

// TODO: Add TCP transformation
public class MovableObject : Model
{
    public Transformation3D<ModelCsPoint> Position { get; private set; }

    public List<MarkPoint<ModelCsPoint>> VisiblePoints { get; private set; }

    public List<ModelCsPoint> MeasuredPoints { get; private set; }

    public double FitQuality { get; private set; }

    public MovableObject(Model model) : base(model.ModelName, model.Points, model.TCP)
    {
        Angle zeroAngle = Angle.FromDegrees(0);
        // TODO: get angle notation from global parameters
        EulerAngleConvention convention = EulerAngleConvention.XYZ;
        Rotation rotation = new(new EulerAngles(zeroAngle, zeroAngle, zeroAngle), convention);
        ModelCsPoint translation = new(0, 0, 0);
        Position = new Transformation3D<ModelCsPoint>(rotation, translation);
        VisiblePoints = new();
        MeasuredPoints = new();
    }

    public void SetClassificationResult(PointCloudClassificationResult<ModelCsPoint> computedTransformation)
    {
        Matrix<double> transformation = computedTransformation.PointCloudTransformation;
        List<ModelCsPoint> foundPoints = computedTransformation.UsedTargetPoints;
        List<ModelCsPoint> visibleInitialPoints = computedTransformation.VisibleInitialPoints;
        if (foundPoints.Count == 0)
        {
            return;
        }

        SetTransformation(transformation);
        SetMeasuredPoints(visibleInitialPoints, foundPoints);
    }

    public void SetMeasuredPoints(List<ModelCsPoint> visibleInitialPoints, List<ModelCsPoint> measuredPoints)
    {
        VisiblePoints = new();
        MeasuredPoints = measuredPoints;
        foreach (ModelCsPoint initialPoint in visibleInitialPoints)
        {
            VisiblePoints.AddRange(from MarkPoint<ModelCsPoint> modelPoint in Points
                                   where initialPoint.Equals(modelPoint.Point)
                                   select modelPoint);
        }
    }

    public void SetTransformation(Matrix<double> transformation)
    {
        if (transformation == null || transformation.ColumnCount != 4 || transformation.RowCount != 4) 
        {
            throw new ArgumentException("Transformation matrix should be 4x4 not null matrix of homography");
        }

        // TODO: get angle notation from global parameters
        EulerAngleConvention convention = Position.Rotation.Convention;
        Matrix<double> rotationMatix = transformation.SubMatrix(0, 3, 0, 3);
        Angle zeroAngle = Angle.FromDegrees(0);
        Rotation rotation = new(new EulerAngles(zeroAngle, zeroAngle, zeroAngle), convention);
        try
        {
            rotation = Rotation.FromRotationMatrix(rotationMatix, convention);
        }
        catch(Exception ex)
        {
            // TODO: how react on situation if rotation matrix cannot be parsed into angles???
            ;
        }

        ModelCsPoint translation = new(transformation[0, 3], transformation[1, 3], transformation[2, 3]);
        Position = new Transformation3D<ModelCsPoint>(rotation, translation);
    }

    public List<MarkPoint<ModelCsPoint>> GetTransformedModelPoints()
    {
        List<MarkPoint<ModelCsPoint>> result = new();
        foreach(MarkPoint<ModelCsPoint> point in Points)
        {
            Vector<double> newCoordinate = Position.TransformationMatrix * point.Point.Homogenous;
            result.Add(point.UpdateCoordinate(new ModelCsPoint(newCoordinate[0], newCoordinate[1], newCoordinate[2])));
        }

        return result;
    }

    private static (List<MarkPoint<T>> left, List<MarkPoint<T>> right) MapMarkPoints<T>(MarkPoint<T>[] leftMarks, MarkPoint<T>[] rightMarks)
        where T : IPoint
    {
        List<MarkPoint<T>> left = leftMarks.Where(i => i.HasCode).OrderBy(i => i.MarkCode.Code).ToList();
        List<MarkPoint<T>> right = rightMarks.Where(i => i.HasCode).OrderBy(i => i.MarkCode.Code).ToList();

        List<int> intersectedCodes = left.Select(i => i.MarkCode.Code).Intersect(right.Select(j => j.MarkCode.Code)).ToList();

        List<MarkPoint<T>> leftResult = left.Where(i => intersectedCodes.Contains(i.MarkCode.Code)).ToList();
        List<MarkPoint<T>> rightResult = right.Where(i => intersectedCodes.Contains(i.MarkCode.Code)).ToList();

        return (leftResult, rightResult);
    }
    
    public override bool Equals(object? obj) => obj is Model model && (ReferenceEquals(this, obj) || Equals(model, 1e-5));
    
    // TODO: Review Models equality
    public bool Equals(Model b, double precition)
    {
        if (ModelName != b.ModelName) 
        {
            return false;
        }

        (List<MarkPoint<ModelCsPoint>> aMarkPoints, List<MarkPoint<ModelCsPoint>> bMarkPoints) = MapMarkPoints(Points, b.Points);
        if (aMarkPoints.Count != Points.Length || bMarkPoints.Count != b.Points.Length) 
        {
            return false;
        }

        for (int i = 0; i < aMarkPoints.Count; i++) 
        {
            if (!aMarkPoints[i].Point.Equals(bMarkPoints[i].Point, precition)) 
            {
                return false;
            }
        }

        return true;
    }
}
