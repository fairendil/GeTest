using DigitalAssembly.Math.Common;

namespace DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;

public class ModelCsPoint : Point3D<ModelCsPoint>
{
    public ModelCsPoint(double X, double Y, double Z)
        : base(X, Y, Z)
    {
    }
}
