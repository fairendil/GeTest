namespace DigitalAssembly.Photogrammetry.Camera;

public sealed class Size
{
    public double Width { get; }
    public double Height { get; }

    public Size(double width, double height)
    {
        Width = width;
        Height = height;
    }
}
