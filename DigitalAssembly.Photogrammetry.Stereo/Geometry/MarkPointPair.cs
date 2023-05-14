using DigitalAssembly.Math.Common.Interfaces;

namespace DigitalAssembly.Photogrammetry.Stereo.Geometry;

public class MarkPointPair<T>
    where T: IPoint
{
    public MarkCode MarkCode { get; }
    public T LeftPoint { get; }
    public T RightPoint { get; }

    public MarkPointPair(MarkCode code, T leftPoint, T rightPoint)
    {
        MarkCode = code;
        LeftPoint = leftPoint;
        RightPoint = rightPoint;
    }
}
