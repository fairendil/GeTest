using DigitalAssembly.Math.Common;

namespace DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;

public class HomogeneousPictureCsPoint: Point3D<HomogeneousPictureCsPoint>
{
    public HomogeneousPictureCsPoint(double x, double y, double w): base(x, y, w) { }
}
