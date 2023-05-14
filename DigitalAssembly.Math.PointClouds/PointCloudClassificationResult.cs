using DigitalAssembly.Math.Common;
using DigitalAssembly.Math.PointClouds.Classification;
using MathNet.Numerics.LinearAlgebra;

namespace DigitalAssembly.Math.PointClouds;

public sealed class PointCloudClassificationResult<PT>
    where PT: Point3D<PT>
{
    public PointCloudClassificationResult(Matrix<double> pointCloudTransformation, List<PT> usedTargetPoints, List<PT> visibleInitialPoints, double fittingQuality, ClassificationStatus status)
    {
        PointCloudTransformation = pointCloudTransformation;
        UsedTargetPoints = usedTargetPoints;
        VisibleInitialPoints = visibleInitialPoints;
        FittingQuality = fittingQuality;
        ClassificationStatus = status;
    }

    public Matrix<double> PointCloudTransformation { get; }

    public List<PT> UsedTargetPoints { get; }
    
    public List<PT> VisibleInitialPoints { get; }

    public double FittingQuality { get; }

    public ClassificationStatus ClassificationStatus { get; }
}
