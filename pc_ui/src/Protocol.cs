using System;
using System.Collections.Generic;
using System.Text;

namespace FpgaControl
{
    public sealed class Frame
    {
        public byte Version, Sequence, Command;
        public byte[] Payload, Wire;
        public long RequestElapsedMs;
    }

    public static class Protocol
    {
        public const byte SetBrightness = 0x11, GetBrightness = 0x12;
        public const ushort BrightnessCapability = 0x0010, ImageAutoCapability = 0x0020;
        public const byte SetImageAuto = 0x13, GetImageAuto = 0x14;
        public static readonly string[] Algorithms = { "原图", "边缘检测", "灰度", "区域填充", "OSD", "圆心检测与跟踪" };
        public static ushort Crc(byte[] data, int offset, int count)
        {
            int value = 0xffff;
            for (int i = offset; i < offset + count; i++)
            {
                value ^= data[i] << 8;
                for (int b = 0; b < 8; b++) value = ((value << 1) ^ ((value & 0x8000) != 0 ? 0x1021 : 0)) & 0xffff;
            }
            return (ushort)value;
        }
        public static byte[] Encode(byte seq, byte cmd, byte[] payload)
        {
            if (payload.Length > 32) throw new ArgumentException("载荷超过 32 字节");
            byte[] data = new byte[payload.Length + 8];
            data[0] = 0xaa; data[1] = 0x55; data[2] = 1; data[3] = seq; data[4] = cmd; data[5] = (byte)payload.Length;
            Array.Copy(payload, 0, data, 6, payload.Length);
            ushort crc = Crc(data, 2, data.Length - 4);
            data[data.Length - 2] = (byte)crc; data[data.Length - 1] = (byte)(crc >> 8);
            return data;
        }
        public static string Hex(byte[] data) { return BitConverter.ToString(data).Replace('-', ' '); }
        public static string AlgorithmName(int id) { return id >= 0 && id < 6 ? Algorithms[id] : "未知 (" + id + ")"; }
        public static void CheckResult(Frame frame)
        {
            if (frame.Payload.Length == 0) throw new InvalidOperationException("响应缺少 RESULT");
            if (frame.Payload[0] != 0) throw new CommandRejected(frame.Payload[0]);
        }
        public static BoardInfo ReadPing(Frame f)
        {
            CheckResult(f);
            if (f.Payload.Length != 8 || f.Payload[1] != 1) throw new InvalidOperationException("PING 响应格式或版本不正确");
            return new BoardInfo { Capabilities = BitConverter.ToUInt16(f.Payload, 2), Width = BitConverter.ToUInt16(f.Payload, 4), Height = BitConverter.ToUInt16(f.Payload, 6) };
        }
        public static byte[] BrightnessPayload(int value)
        {
            if (value < -128 || value > 127) throw new ArgumentOutOfRangeException("value");
            return new byte[] { unchecked((byte)(sbyte)value) };
        }
        public static int ReadBrightness(Frame f)
        {
            CheckResult(f);
            if ((f.Command != 0x91 && f.Command != 0x92) || f.Payload.Length != 2)
                throw new InvalidOperationException("亮度响应格式不正确");
            return unchecked((sbyte)f.Payload[1]);
        }
        public static byte ImageAutoPayload(bool awb, bool ae) { return (byte)((awb ? 1 : 0) | (ae ? 2 : 0)); }
        public static byte ReadImageAutoAck(Frame f)
        {
            CheckResult(f);
            if (f.Command != 0x93 || f.Payload.Length != 2 || f.Payload[1] > 3)
                throw new InvalidOperationException("图像自动调节确认格式不正确");
            return f.Payload[1];
        }
        public static ImageAutoStatus ReadImageAuto(Frame f)
        {
            CheckResult(f);
            if (f.Command != 0x94 || f.Payload.Length != 16 || f.Payload[2] > 6 || (f.Payload[1] & 0xe0) != 0)
                throw new InvalidOperationException("图像自动调节状态格式不正确");
            uint r=BitConverter.ToUInt32(f.Payload,8), b=BitConverter.ToUInt32(f.Payload,12);
            if (r>0xfffff || b>0xfffff) throw new InvalidOperationException("白平衡增益超出范围");
            return new ImageAutoStatus { Flags=f.Payload[1], State=f.Payload[2], Mean=f.Payload[3],
                Target=f.Payload[4], Gain=f.Payload[5], Exposure=BitConverter.ToUInt16(f.Payload,6),
                RedGain=r, BlueGain=b, TransportAge=f.RequestElapsedMs };
        }
        public static BoardStatus ReadStatus(Frame f)
        {
            CheckResult(f);
            byte[] p = f.Payload;
            if (p.Length != 28 || p[1] > 5 || (p[2] > 5 && p[2] != 255)) throw new InvalidOperationException("STATUS 响应格式不正确");
            return new BoardStatus {
                Algorithm = p[1], PendingAlgorithm = p[2], Flags = p[3], Width = BitConverter.ToUInt16(p, 4), Height = BitConverter.ToUInt16(p, 6),
                FrameId = BitConverter.ToUInt32(p, 8), SampleId = BitConverter.ToUInt32(p, 12), X = BitConverter.ToUInt16(p, 16), Y = BitConverter.ToUInt16(p, 18),
                SampleAge = BitConverter.ToUInt16(p, 20), StreamPeriod = BitConverter.ToUInt16(p, 22), Uptime = BitConverter.ToUInt32(p, 24), TransportAge = f.RequestElapsedMs
            };
        }
    }

    public sealed class CommandRejected : Exception
    {
        public readonly byte Code;
        public CommandRejected(byte code) : base(MessageFor(code)) { Code = code; }
        static string MessageFor(byte code)
        {
            string[] text = { "成功", "功能未实现 (UNSUPPORTED)", "命令长度错误 (BAD_LENGTH)", "参数错误 (BAD_VALUE)", "FPGA 正在应用设置，请稍后重试 (BUSY)", "协议版本不匹配 (BAD_VERSION)", "帧边界应用超时，请检查视频时钟 (APPLY_TIMEOUT)", "命令序号冲突 (SEQ_CONFLICT)" };
            return code < text.Length ? text[code] : "FPGA 错误码 " + code;
        }
    }
    public sealed class BoardInfo
    {
        public ushort Capabilities, Width, Height;
        public bool SupportsImageAuto { get { return (Capabilities & Protocol.ImageAutoCapability) != 0; } }
        public bool SupportsBrightness { get { return (Capabilities & Protocol.BrightnessCapability) != 0; } }
    }
    public sealed class ImageAutoStatus
    {
        public byte Flags, State, Mean, Target, Gain;
        public ushort Exposure;
        public uint RedGain, BlueGain;
        public long TransportAge;
        public bool Awb { get { return (Flags & 1)!=0; } }
        public bool Ae { get { return (Flags & 2)!=0; } }
        public bool MeanValid { get { return (Flags & 4)!=0 && Fresh; } }
        public bool SensorValid { get { return (Flags & 8)!=0 && Fresh; } }
        public bool Fresh { get { return (Flags & 16)!=0 && TransportAge>=0 && TransportAge<1500; } }
        public string StateText { get { return new string[] { "等待亮度统计", "正在调节", "亮度稳定", "暗场已到上限", "亮场已到下限", "曝光保持", "摄像头配置故障" }[State]; } }
    }
    public sealed class BoardStatus
    {
        public byte Algorithm, PendingAlgorithm, Flags;
        public ushort Width, Height, X, Y, SampleAge, StreamPeriod;
        public uint FrameId, SampleId, Uptime;
        public long TransportAge;
        public bool VideoAlive { get { return (Flags & 1) != 0; } }
        public bool ResultValid { get { return (Flags & 2) != 0; } }
        public bool TrackingEnabled { get { return (Flags & 4) != 0; } }
        public bool UartOwner { get { return (Flags & 8) != 0; } }
        public bool SwitchPending { get { return (Flags & 16) != 0; } }
        public bool CoordinateUsable(long elapsedMs)
        {
            return elapsedMs >= 0 && TransportAge >= 0 && elapsedMs + TransportAge < 1500 && Algorithm == 5 && VideoAlive && ResultValid && !SwitchPending &&
                SampleAge != 65535 && (long)SampleAge + TransportAge + elapsedMs < 250 && X < Width && Y < Height;
        }
    }

    // One byte at a time keeps candidate memory bounded and supports fragmented USB reads.
    public sealed class StreamDecoder
    {
        readonly List<byte> buffer = new List<byte>(40);
        long lastByte = -1;
        public void Expire(long now) { if (lastByte >= 0 && now - lastByte > 100) { buffer.Clear(); lastByte = -1; } }
        public Frame Feed(byte value, long now)
        {
            Expire(now); lastByte = now; buffer.Add(value);
            while (buffer.Count >= 2)
            {
                if (buffer[0] != 0xaa || buffer[1] != 0x55) { buffer.RemoveAt(0); continue; }
                if (buffer.Count < 6) return null;
                if (buffer[5] > 32) { buffer.RemoveAt(0); continue; }
                int size = 8 + buffer[5];
                if (buffer.Count < size) return null;
                byte[] wire = buffer.GetRange(0, size).ToArray();
                ushort crc = (ushort)(wire[size - 2] | wire[size - 1] << 8);
                if (crc != Protocol.Crc(wire, 2, size - 4)) { buffer.RemoveAt(0); continue; }
                buffer.RemoveRange(0, size);
                byte[] payload = new byte[wire[5]]; Array.Copy(wire, 6, payload, 0, payload.Length);
                return new Frame { Version = wire[2], Sequence = wire[3], Command = wire[4], Payload = payload, Wire = wire };
            }
            return null;
        }
    }
}
