using DigitalAssembly.Math.Common.Interfaces;
using System.Diagnostics;

namespace DigitalAssembly.Photogrammetry;

[DebuggerDisplay("[{MarkCode}]: {Point}")]
public class MarkPoint<T>
    where T: IPoint
{
    public MarkPoint(MarkCode code, T point)
    {
        MarkCode = code;
        Point = point;
        HasCode = code.Type != MarkCodeType.Uncoded;
    }

    public MarkPoint<PT> UpdateCoordinate<PT>(PT point)
        where PT : IPoint => new(MarkCode, point);

    public static MarkPoint<TPoint> Uncoded<TPoint>(TPoint point) where TPoint : IPoint 
        => new(new MarkCode(0, MarkCodeType.Uncoded), point);

    public static MarkPoint<TPoint> FromCode<TPoint>(int code, MarkCodeType type, TPoint point)
        where TPoint : IPoint
    {
        return type switch
        {
            MarkCodeType.Uncoded    => new MarkPoint<TPoint>(new MarkCode(0, type), point),
            MarkCodeType.BitCode14b => code > 0 ? new MarkPoint<TPoint>(new MarkCode(code, type), point) :
                                                  MarkPoint<TPoint>.Uncoded(point),
            _                       => throw new NotImplementedException(),
        };
    }

    public bool HasCode { get; }

    public MarkCode MarkCode { get; }

    public T Point { get; }

    public override string ToString()
    {
        return $"[{MarkCode}]: {Point}";
    }
}
