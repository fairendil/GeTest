using DigitalAssembly.Photogrammetry.Geometry.CoordinateSystems;
using DigitalAssembly.Photogrammetry;

namespace DigitalAssembly.GoldenEye.DB;

// TODO: automapper
public class Model
{
    public string ModelName { get; private set; }

    public MarkPoint<ModelCsPoint>[] Points { get; private set; }

    public List<ModelCsPoint> TCP { get; private set; }

    protected Model(string Name, MarkPoint<ModelCsPoint>[] points, List<ModelCsPoint> tcp)
    {
        ModelName = Name;
        Points = points;
        TCP = tcp;
    }
}
