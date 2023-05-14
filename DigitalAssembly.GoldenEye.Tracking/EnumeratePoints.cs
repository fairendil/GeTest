using DigitalAssembly.Math.Common;
using DigitalAssembly.Photogrammetry;

namespace DigitalAssembly.GoldenEye.Tracking;

public sealed class EnumeratePoints<T>
    where T : Point3D<T>
{
    private bool _IsTrackingOn;
    private readonly int _StartIndex;
    private List<MarkPoint<T>> _PreviousPoints;

    public EnumeratePoints() 
    {
        _IsTrackingOn = false;
        _StartIndex = 1000;
        _PreviousPoints = new();
    }

    public void SetTrackingMode(bool isTrackingOn) => _IsTrackingOn = isTrackingOn;

    private List<MarkPoint<T>> TrackPoints(List<MarkPoint<T>> intiialPoints) => throw new NotImplementedException();

    public List<MarkPoint<T>> Enumerate(List<MarkPoint<T>> initialPoints)
    {
        List<MarkPoint<T>> result = new();
        if (_IsTrackingOn)
        {
            initialPoints = TrackPoints(initialPoints);
            _PreviousPoints = initialPoints;
        }

        int index = _StartIndex;
        foreach (MarkPoint<T> point in initialPoints)
        {
            MarkPoint<T> resultPoint = point;
            if (point.MarkCode.Type == MarkCodeType.Uncoded)
            {
                resultPoint = new MarkPoint<T>(new MarkCode(index, MarkCodeType.Uncoded), resultPoint.Point);
                index++;
            }

            result.Add(resultPoint);
        }

        return result;
    }
}
