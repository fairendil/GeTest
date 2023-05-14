using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using DigitalAssembly.Photogrammetry;

namespace DigitalAssembly.GoldenEye.DB;

public sealed class ReferenceModel: Model
{
    public ReferenceModel(string name, MarkPoint<ModelCsPoint>[] points, List<ModelCsPoint> tcp) : base(name, points, tcp)
    {
    }
}
