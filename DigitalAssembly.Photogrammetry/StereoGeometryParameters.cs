namespace DigitalAssembly.Photogrammetry;

public class StereoGeometryParameters
{
    private readonly double _nearestDistance, _farthesDistance;

    public (double Nearest, double Farthest) WorkingSpace => (_nearestDistance, _farthesDistance);

    public double Precision { get; }

    public double EpipolarEpsilon { get; set; }

    public StereoGeometryParameters(double nearDistance, double farDistance, double epipolarDistance, double precision)
    {
        _nearestDistance = nearDistance;
        _farthesDistance = farDistance;
        EpipolarEpsilon = epipolarDistance;
        Precision = precision;
    }
}
