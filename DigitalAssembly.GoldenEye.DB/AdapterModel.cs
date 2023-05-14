using DigitalAssembly.Photogrammetry;
using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;

namespace DigitalAssembly.GoldenEye.DB;

public sealed class AdapterModel: Model
{
    public AdapterModel(string name, MarkPoint<ModelCsPoint>[] points, List<ModelCsPoint> tcp): base(name, points, tcp)
    {
    }
}
