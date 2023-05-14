namespace DigitalAssembly.Photogrammetry;

public class MarkCode
{
    public MarkCode(int code, MarkCodeType type)
    {
        Code = code;
        Type = type;
    }

    public int Code { get; set; }
    public MarkCodeType Type { get; }

    public override string ToString() => $"{Code}";

    public override bool Equals(object? obj)
    {
        return obj != null && (ReferenceEquals(this, obj) ||
               (obj is MarkCode anotherCode && Type == anotherCode.Type && Code == anotherCode.Code));
    }
}
