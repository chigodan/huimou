using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Threading;

namespace FpgaControl
{
    public interface ITransport : IDisposable
    {
        void Write(byte[] packet);
        int ReadByte();
    }
    public sealed class SerialTransport : ITransport
    {
        readonly SerialPort port;
        public SerialTransport(string name)
        {
            port = new SerialPort(name, 115200, Parity.None, 8, StopBits.One);
            port.Handshake = Handshake.None; port.DtrEnable = false; port.RtsEnable = false;
            port.ReadTimeout = 30; port.WriteTimeout = 1000;
            try { port.Open(); port.DiscardInBuffer(); }
            catch { port.Dispose(); throw; }
        }
        public void Write(byte[] packet) { port.Write(packet, 0, packet.Length); }
        public int ReadByte() { return port.ReadByte(); }
        public void Dispose() { port.Dispose(); }
    }
    public sealed class Exchange
    {
        readonly ITransport transport;
        readonly Func<bool> stopped;
        readonly Action<string> log;
        readonly Action<byte> saveSequence;
        readonly Stopwatch clock = Stopwatch.StartNew();
        readonly StreamDecoder decoder = new StreamDecoder();
        int sequence;
        public Exchange(ITransport transport, Func<bool> stopped, Action<string> log, byte previous, Action<byte> save)
        { this.transport = transport; this.stopped = stopped; this.log = log; sequence = previous; saveSequence = save; }
        public Frame Request(byte cmd, byte[] payload, bool verbose, int timeoutMs)
        {
            // A conflict rejects the request without executing it. Allocate one fresh sequence on conflict.
            for (int conflict = 0; conflict < 2; conflict++)
            {
                sequence = sequence % 255 + 1; saveSequence((byte)sequence);
                byte[] packet = Protocol.Encode((byte)sequence, cmd, payload);
                long requestStart = clock.ElapsedMilliseconds;
                for (int attempt = 0; attempt < 3; attempt++)
                {
                    if (stopped()) throw new OperationCanceledException();
                    if (verbose) log("TX  " + Protocol.Hex(packet));
                    transport.Write(packet);
                    long start = clock.ElapsedMilliseconds;
                    while (clock.ElapsedMilliseconds - start < timeoutMs)
                    {
                        if (stopped()) throw new OperationCanceledException();
                        int value;
                        try { value = transport.ReadByte(); }
                        catch (TimeoutException) { decoder.Expire(clock.ElapsedMilliseconds); continue; }
                        Frame frame = decoder.Feed((byte)value, clock.ElapsedMilliseconds);
                        if (frame == null || frame.Version != 1 || frame.Sequence != sequence || frame.Command != (cmd | 0x80)) continue;
                        if (verbose) log("RX  " + Protocol.Hex(frame.Wire));
                        if (frame.Payload.Length == 1 && frame.Payload[0] == 7 && conflict == 0) { log("序号冲突，使用新序号重新发送。"); goto NextSequence; }
                        Protocol.CheckResult(frame);
                        // Retries may receive a cached snapshot. Include the full request duration.
                        frame.RequestElapsedMs = clock.ElapsedMilliseconds - requestStart;
                        return frame;
                    }
                    log("等待响应超时：第 " + (attempt + 1) + "/3 次，同序号重发。");
                }
                throw new TimeoutException("三次请求均未收到有效响应，状态未确认。请检查 USB、COM 口和控制固件后重新连接。");
            NextSequence: ;
            }
            throw new CommandRejected(7);
        }
    }
    public sealed class Command { public byte Code, Value; }

    public sealed class SerialController : IDisposable
    {
        readonly ConcurrentQueue<Command> queue = new ConcurrentQueue<Command>();
        readonly AutoResetEvent wake = new AutoResetEvent(false);
        readonly object lifecycle = new object();
        bool workerClosed;
        readonly Thread worker;
        readonly string portName;
        readonly bool demo;
        volatile bool stop;
        public volatile int PollMs = 200;
        public volatile bool Verbose;
        public Action<string> Log;
        public Action<BoardInfo> Connected;
        public Action<BoardStatus> Status;
        public Action<byte, bool, string> AlgorithmCompleted;
        public Action<int> BrightnessChanged;
        public Action<int, bool, string> BrightnessCompleted;
        public Action<ImageAutoStatus> ImageAutoChanged;
        public Action<byte, bool, string> ImageAutoCompleted;
        public Action<string> Ended;
        public SerialController(string portName, bool demo)
        {
            this.portName = portName; this.demo = demo;
            worker = new Thread(Run); worker.IsBackground = true; worker.Name = "FPGA UART worker";
        }
        public void Start() { worker.Start(); }
        public void Send(byte command, byte value) { lock (lifecycle) { if (!stop && !workerClosed) { queue.Enqueue(new Command { Code = command, Value = value }); wake.Set(); } } }
        void Emit(string message) { if (Log != null && !stop) Log(message); }
        public void Dispose() { lock (lifecycle) { stop = true; if (!workerClosed) wake.Set(); } /* Worker owns and closes its serial port. */ }
        void Run()
        {
            string reason = "已断开";
            try
            {
                using (ITransport transport = demo ? (ITransport)new DemoTransport() : new SerialTransport(portName))
                {
                    string sequencePath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, ".ui_seq_" + (demo ? "demo" : portName) + ".txt");
                    int previous = 0;
                    if (!demo && File.Exists(sequencePath)) int.TryParse(File.ReadAllText(sequencePath).Trim(), out previous);
                    if (previous < 0 || previous > 255) previous = 0;
                    Exchange exchange = new Exchange(transport, delegate { return stop; }, Emit, (byte)previous,
                        delegate(byte seq) { if (!demo) File.WriteAllText(sequencePath, seq.ToString()); });
                    BoardInfo info = Protocol.ReadPing(exchange.Request(1, new byte[0], true, 1000));
                    if ((info.Capabilities & 2) == 0) throw new InvalidOperationException("固件未提供状态查询功能。");
                    if (stop) return;
                    if (Connected != null) Connected(info);
                    Stopwatch clock = Stopwatch.StartNew(); long nextPoll = 0;
                    while (!stop)
                    {
                        Command cmd;
                        bool userCommand = queue.TryDequeue(out cmd);
                        if (!userCommand && clock.ElapsedMilliseconds < nextPoll) { wake.WaitOne(30); continue; }
                        if (!userCommand) cmd = new Command { Code = 2 };
                        try
                        {
                            Frame reply = exchange.Request(cmd.Code, cmd.Code == 16 || cmd.Code == Protocol.SetBrightness || cmd.Code == Protocol.SetImageAuto ? new byte[] { cmd.Value } : new byte[0], userCommand || Verbose, 1000);
                            if (cmd.Code == 16)
                            {
                                if (reply.Payload.Length != 2 || reply.Payload[1] != cmd.Value) throw new InvalidOperationException("切换确认与请求不一致，状态未确认。");
                                Emit("已确认生效：" + Protocol.AlgorithmName(cmd.Value));
                                if (AlgorithmCompleted != null) AlgorithmCompleted(cmd.Value, true, "切换成功");
                                nextPoll = 0;
                            }
                            else if (cmd.Code == Protocol.SetBrightness)
                            {
                                int value = Protocol.ReadBrightness(reply);
                                if (value != unchecked((sbyte)cmd.Value)) throw new InvalidOperationException("亮度确认与请求不一致。");
                                Emit("显示亮度已确认：" + value);
                                if (BrightnessChanged != null) BrightnessChanged(value);
                                if (BrightnessCompleted != null) BrightnessCompleted(value, true, "亮度已生效");
                                nextPoll = 0;
                            }
                            else if (cmd.Code == Protocol.SetImageAuto)
                            {
                                byte flags=Protocol.ReadImageAutoAck(reply);
                                if(flags!=cmd.Value) throw new InvalidOperationException("图像自动调节确认与请求不一致");
                                Emit("图像设置已确认：AWB " + ((flags&1)!=0 ? "开" : "关") + "，AE " + ((flags&2)!=0 ? "开" : "关"));
                                if(ImageAutoCompleted!=null) ImageAutoCompleted(flags,true,"图像设置已生效");
                                nextPoll=0;
                            }
                            else if (cmd.Code == 1)
                            {
                                Protocol.ReadPing(reply); Emit("PING 成功"); nextPoll = 0;
                            }
                            else
                            {
                                BoardStatus snapshot = Protocol.ReadStatus(reply);
                                if (Status != null) Status(snapshot);
                                if (info.SupportsBrightness)
                                {
                                    int value = Protocol.ReadBrightness(exchange.Request(Protocol.GetBrightness, new byte[0], Verbose, 1000));
                                    if (BrightnessChanged != null) BrightnessChanged(value);
                                }
                                if(info.SupportsImageAuto)
                                {
                                    ImageAutoStatus image=Protocol.ReadImageAuto(exchange.Request(Protocol.GetImageAuto,new byte[0],Verbose,1000));
                                    if(ImageAutoChanged!=null) ImageAutoChanged(image);
                                }
                                nextPoll = clock.ElapsedMilliseconds + PollMs;
                            }
                        }
                        catch (CommandRejected error)
                        {
                            Emit(error.Message);
                            if (cmd.Code == 16 && AlgorithmCompleted != null) AlgorithmCompleted(cmd.Value, false, error.Message);
                            if (cmd.Code == Protocol.SetBrightness && BrightnessCompleted != null) BrightnessCompleted(unchecked((sbyte)cmd.Value), false, error.Message);
                            if(cmd.Code==Protocol.SetImageAuto && ImageAutoCompleted!=null) ImageAutoCompleted(cmd.Value,false,error.Message);
                            if (cmd.Code == 2) { reason = "状态查询失败：" + error.Message; break; }
                            nextPoll = 0;
                        }
                    }
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception error) { reason = error.Message; Emit(reason); }
            finally { lock (lifecycle) { workerClosed = true; wake.Dispose(); } if (Ended != null) Ended(reason); }
        }
    }
    // Demo travels through the same wire codec, without opening any COM port.
    public sealed class DemoTransport : ITransport
    {
        readonly System.Collections.Generic.Queue<byte> incoming = new System.Collections.Generic.Queue<byte>();
        readonly Stopwatch timer = Stopwatch.StartNew(); byte algorithm, brightness, imageFlags=3; bool owner; uint sample;
        public void Write(byte[] packet)
        {
            byte[] payload;
            if (packet[4] == 1) payload = new byte[] { 0, 1, 59, 0, 0, 5, 208, 2 };
            else if (packet[4] == 16) { algorithm = packet[6]; owner = true; payload = new byte[] { 0, algorithm }; }
            else if (packet[4] == Protocol.SetBrightness)
            {
                if (packet[5] != 1) payload = new byte[] { 2 };
                else { brightness = packet[6]; payload = new byte[] { 0, brightness }; }
            }
            else if(packet[4]==Protocol.SetImageAuto)
            {
                if(packet[5]!=1) payload=new byte[]{2};
                else if(packet[6]>3) payload=new byte[]{3};
                else { imageFlags=packet[6];payload=new byte[]{0,imageFlags}; }
            }
            else if(packet[4]==Protocol.GetImageAuto)
            {
                if(packet[5]!=0) payload=new byte[]{2};
                else
                {
                    payload=new byte[16];payload[1]=(byte)(imageFlags|28);
                    payload[2]=(byte)((imageFlags&2)!=0 ? 2 : 5);payload[3]=128;payload[4]=128;payload[5]=48;
                    Put(payload,6,BitConverter.GetBytes((ushort)1600));
                    Put(payload,8,BitConverter.GetBytes((uint)((imageFlags&1)!=0 ? 0x12000 : 0x10000)));
                    Put(payload,12,BitConverter.GetBytes((uint)((imageFlags&1)!=0 ? 0xe000 : 0x10000)));
                }
            }
            else if (packet[4] == Protocol.GetBrightness)
                payload = packet[5] == 0 ? new byte[] { 0, brightness } : new byte[] { 2 };
            else
            {
                payload = new byte[28]; payload[1] = algorithm; payload[2] = 255;
                bool circle = algorithm == 5; bool found = circle && (timer.ElapsedMilliseconds / 5000) % 4 != 3;
                payload[3] = (byte)(1 | (found ? 2 : 0) | (circle ? 4 : 0) | (owner ? 8 : 0));
                Put(payload, 4, BitConverter.GetBytes((ushort)1280)); Put(payload, 6, BitConverter.GetBytes((ushort)720));
                Put(payload, 8, BitConverter.GetBytes((uint)(timer.ElapsedMilliseconds * 60 / 1000)));
                if (circle) sample++; Put(payload, 12, BitConverter.GetBytes(sample));
                Put(payload, 16, BitConverter.GetBytes(found ? (ushort)(640 + 320 * Math.Sin(timer.ElapsedMilliseconds / 1300.0)) : (ushort)65535));
                Put(payload, 18, BitConverter.GetBytes(found ? (ushort)(360 + 170 * Math.Cos(timer.ElapsedMilliseconds / 1700.0)) : (ushort)65535));
                Put(payload, 20, BitConverter.GetBytes(circle ? (ushort)10 : (ushort)65535)); Put(payload, 24, BitConverter.GetBytes((uint)timer.ElapsedMilliseconds));
            }
            foreach (byte value in Protocol.Encode(packet[3], (byte)(packet[4] | 0x80), payload)) incoming.Enqueue(value);
        }
        static void Put(byte[] buffer, int offset, byte[] value) { Array.Copy(value, 0, buffer, offset, value.Length); }
        public int ReadByte() { if (incoming.Count != 0) return incoming.Dequeue(); Thread.Sleep(10); throw new TimeoutException(); }
        public void Dispose() { }
    }
}
