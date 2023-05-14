using System;

namespace DigitalAssembly.Math.Matrices;

public static partial class StaticMatrix
{
    public enum TypeTransform
        {
            MatrixTransform,
            EulerXZX,
            EulerXYX,
            EulerYXY,
            EulerYZY,
            EulerZYZ,
            EulerZXZ,
            EulerXZY,
            EulerXYZ,
            EulerYXZ,
            EulerYZX,
            EulerZYX,
            EulerZXY,
            AxisAngle,
            Quaternion,
            DualQuaternion,
            ClassicDH,
            ModernDH,
            ExtendedDH
		}

    public static TypeTransform FromNotationAngles(this NotationAngles notationAngles)
    {
        switch (notationAngles)
        {
            case (NotationAngles.XYX):
                return TypeTransform.EulerXYX;
            case (NotationAngles.XZX):
                return TypeTransform.EulerXZX;
            case (NotationAngles.YXY):
                return TypeTransform.EulerYXY;
            case (NotationAngles.YZY):
                return TypeTransform.EulerYZY;
            case (NotationAngles.ZXZ):
                return TypeTransform.EulerZXZ;
            case (NotationAngles.ZYZ):
                return TypeTransform.EulerZYZ;
            case (NotationAngles.XYZ):
                return TypeTransform.EulerXYZ;
            case (NotationAngles.XZY):
                return TypeTransform.EulerXZY;
            case (NotationAngles.YXZ):
                return TypeTransform.EulerYXZ;
            case (NotationAngles.YZX):
                return TypeTransform.EulerYZX;
            case (NotationAngles.ZYX):
                return TypeTransform.EulerZYX;
            case (NotationAngles.ZXY):
                return TypeTransform.EulerZXY;
            default:
                throw new ArgumentException("Wrong Notation angles");
        }
    }
}
