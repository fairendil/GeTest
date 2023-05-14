using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using static System.Math;

namespace DigitalAssembly.Math.Matrices;
public class AxisAngle
{
    public readonly Point3D Axis;
    public readonly Angle Angle;

    public AxisAngle(Point3D axis, Angle angle)
    {
        Axis = axis;
        Angle = angle;
    }

    public AxisAngle()
    {
        Axis = new Point3D();
        Angle = new Angle();
    }

    public Quaternion GetQuaternion()
    {
        double coefTemp = Sin(Angle.Radians / 2) / Angle.Radians;

        Quaternion quatOut = new Quaternion(
            Cos(Angle.Radians / 2),
            Axis.X * coefTemp,
            Axis.Y * coefTemp,
            Axis.Z * coefTemp);

        return quatOut;
    }

    public static AxisAngle FromQuaternion(Quaternion quat)
    {
        Angle angle = Angle.FromRadians(2 * Acos(quat.Real));

        double coef = angle.Radians / Sin(angle.Radians / 2);

        Point3D axis = new Point3D(
            quat.ImagX * coef,
            quat.ImagY * coef,
            quat.ImagZ * coef);

        return new AxisAngle(axis, angle);
    }
}
