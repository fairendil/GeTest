using DigitalAssembly.Math.Common;

namespace DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;

public class CameraCsPoint : Point3D<CameraCsPoint>
{
    public CameraCsPoint(double X, double Y, double Z)
        : base(X, Y, Z)
    {
    }
}