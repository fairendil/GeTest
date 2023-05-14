namespace DigitalAssembly.Photogrammetry.Camera.DistortionModels;

public class ClassicDistortionParameters
{
    public ClassicDistortionParameters(double a1, double a2, double a3, double b1, double b2, double c1, double c2)
    {
        A1 = a1;
        A2 = a2;
        A3 = a3;
        B1 = b1;
        B2 = b2;
        C1 = c1;
        C2 = c2;
    }

    public double[] ClassicDistortion => new double[] { A1, A2, A3, B1, B2, C1, C2 };

    [Obsolete]
    public double[] Radial => new double[] { A1, A2, A3 };

    [Obsolete]
    public double[] Tangential => new double[] { B1, B2 };

    [Obsolete]
    public double[] Affine => new double[] { C1, C2 };

    // TODO: Extract class
    // Radial distortion
    public double A1 { get; }
    public double A2 { get; }
    public double A3 { get; }

    // TODO: Extract class
    // Tangential distortion
    public double B1 { get; }
    public double B2 { get; }

    //TODO: Extract class
    // Affine distortion
    public double C1 { get; }
    public double C2 { get; }
}
