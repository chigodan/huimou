using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Text;
using System.Threading;
using System.Windows.Forms;

namespace FpgaControl
{
    public static class Tests
    {
        static int checks;
        static void Check(bool condition, string description)
        { if (!condition) throw new Exception("FAIL: " + description); checks++; }
        static Frame Decode(byte[] wire)
        {
            StreamDecoder decoder = new StreamDecoder(); Frame result = null;
            foreach (byte b in wire) { Frame f = decoder.Feed(b, 0); if (f != null) result = f; }
            return result;
        }
        static List<Frame> Feed(StreamDecoder decoder, byte[] wire, long time)
        {
            List<Frame> frames = new List<Frame>(); foreach (byte b in wire) { Frame f = decoder.Feed(b, time); if (f != null) frames.Add(f); } return frames;
        }
        sealed class FakeTransport : ITransport
        {
            public readonly List<byte[]> Writes = new List<byte[]>(); readonly Queue<byte> input = new Queue<byte>();
            public Action<FakeTransport, byte[]> Written;
            public void Write(byte[] packet) { Writes.Add((byte[])packet.Clone()); if (Written != null) Written(this, packet); }
            public void Enqueue(byte[] packet) { foreach (byte b in packet) input.Enqueue(b); }
            public int ReadByte() { if (input.Count > 0) return input.Dequeue(); Thread.Sleep(1); throw new TimeoutException(); }
            public void Dispose() { }
        }
        static Exchange Client(FakeTransport t, byte previous)
        { return new Exchange(t, delegate { return false; }, delegate(string s) { }, previous, delegate(byte b) { }); }
        static void CodecTests()
        {
            byte[] standard = Encoding.ASCII.GetBytes("123456789"); Check(Protocol.Crc(standard, 0, standard.Length) == 0x29b1, "CRC known answer");
            Check(Protocol.Hex(Protocol.Encode(5, 16, new byte[] { 2 })) == "AA 55 01 05 10 01 02 08 57", "frozen protocol GRAY request");
            Check(Protocol.Hex(Protocol.Encode(5, 0x90, new byte[] { 0, 2 })) == "AA 55 01 05 90 02 00 02 5A E0", "frozen protocol GRAY ACK");
            byte[] payload = new byte[32]; for (int i = 0; i < payload.Length; i++) payload[i] = (byte)i; payload[9] = 0xaa; payload[10] = 0x55;
            byte[] packet = Protocol.Encode(255, 0x82, payload);
            for (int split = 0; split <= packet.Length; split++)
            {
                StreamDecoder decoder = new StreamDecoder(); int count = 0; Frame f = null;
                for (int i = 0; i < packet.Length; i++) { Frame item = decoder.Feed(packet[i], i < split ? 0 : 20); if (item != null) { count++; f = item; } }
                Check(count == 1 && f.Payload.Length == 32 && f.Payload[9] == 0xaa, "fragmentation " + split);
            }
            byte[] bad = (byte[])packet.Clone(); bad[8] ^= 1;
            StreamDecoder recovery = new StreamDecoder(); Check(Feed(recovery, bad, 0).Count == 0, "bad CRC rejected"); Check(Feed(recovery, packet, 1).Count == 1, "recover after bad CRC");
            recovery = new StreamDecoder(); Feed(recovery, new byte[] { 0xaa, 0x55, 1, 3, 2, 33 }, 0); Check(Feed(recovery, packet, 1).Count == 1, "recover oversized candidate");
            recovery = new StreamDecoder(); Feed(recovery, new byte[] { 0xaa, 0x55, 1, 3, 2, 20 }, 0); recovery.Expire(101); Check(Feed(recovery, packet, 102).Count == 1, "inter-byte gap recovery");
            recovery = new StreamDecoder(); Check(Feed(recovery, packet, 0).Count + Feed(recovery, packet, 0).Count == 2, "consecutive packets");
        }
        static void ExchangeTests()
        {
            FakeTransport t = new FakeTransport(); t.Written = delegate(FakeTransport a, byte[] p) {
                a.Enqueue(p); // Request echo cannot acknowledge.
                a.Enqueue(Protocol.Encode(0, 0xe0, new byte[28]));
                a.Enqueue(Protocol.Encode((byte)(p[3] + 1), 0x90, new byte[] { 0, 2 }));
                a.Enqueue(Protocol.Encode(p[3], 0x82, new byte[28]));
                byte[] bad = Protocol.Encode(p[3], 0x90, new byte[] { 0, 2 }); bad[bad.Length - 1] ^= 1; a.Enqueue(bad);
                byte[] version = Protocol.Encode(p[3], 0x90, new byte[] { 0, 2 }); version[2] = 2;
                ushort crc = Protocol.Crc(version, 2, version.Length - 4); version[version.Length - 2] = (byte)crc; version[version.Length - 1] = (byte)(crc >> 8); a.Enqueue(version);
                a.Enqueue(Protocol.Encode(p[3], 0x90, new byte[] { 0, 2 }));
            };
            Frame reply = Client(t, 4).Request(16, new byte[] { 2 }, false, 30);
            Check(reply.Command == 0x90 && reply.Sequence == 5 && t.Writes.Count == 1, "filter echo/event/sequence/command/version/CRC");
            t = new FakeTransport(); t.Written = delegate(FakeTransport a, byte[] p) { if (a.Writes.Count == 3) a.Enqueue(Protocol.Encode(p[3], 0x81, new byte[] { 0 })); };
            Client(t, 255).Request(1, new byte[0], false, 10);
            Check(t.Writes.Count == 3 && t.Writes[0][3] == 1 && Protocol.Hex(t.Writes[0]) == Protocol.Hex(t.Writes[2]), "timeout retries identical and sequence wraps");
            t = new FakeTransport(); bool timedOut = false; try { Client(t, 0).Request(1, new byte[0], false, 5); } catch (TimeoutException) { timedOut = true; }
            Check(timedOut && t.Writes.Count == 3, "three failed sends terminate");
            t = new FakeTransport(); t.Written = delegate(FakeTransport a, byte[] p) { a.Enqueue(Protocol.Encode(p[3], (byte)(p[4] | 0x80), a.Writes.Count == 1 ? new byte[] { 7 } : new byte[] { 0, 2 })); };
            Client(t, 4).Request(16, new byte[] { 2 }, false, 20); Check(t.Writes.Count == 2 && t.Writes[0][3] == 5 && t.Writes[1][3] == 6, "conflict uses fresh sequence");
            foreach (byte error in new byte[] { 4, 6, 7 })
            {
                t = new FakeTransport(); byte code = error; t.Written = delegate(FakeTransport a, byte[] p) { a.Enqueue(Protocol.Encode(p[3], 0x90, new byte[] { code })); };
                bool rejected = false; try { Client(t, 1).Request(16, new byte[] { 5 }, false, 20); } catch (CommandRejected e) { rejected = e.Code == code; }
                Check(rejected && t.Writes.Count == (error == 7 ? 2 : 1), "business error terminates " + error);
            }
            t = new FakeTransport(); bool cancelled = false; Exchange stopped = new Exchange(t, delegate { return true; }, delegate(string s) { }, 0, delegate(byte b) { });
            try { stopped.Request(1, new byte[0], false, 10); } catch (OperationCanceledException) { cancelled = true; }
            Check(cancelled && t.Writes.Count == 0, "cancel before write");
        }
        static void StatusTests()
        {
            byte[] p = new byte[28]; p[1] = 5; p[2] = 255; p[3] = 15;
            Array.Copy(BitConverter.GetBytes((ushort)1280), 0, p, 4, 2); Array.Copy(BitConverter.GetBytes((ushort)720), 0, p, 6, 2);
            Array.Copy(BitConverter.GetBytes((uint)0x12345678), 0, p, 8, 4); Array.Copy(BitConverter.GetBytes((uint)43), 0, p, 12, 4);
            Array.Copy(BitConverter.GetBytes((ushort)1279), 0, p, 16, 2); Array.Copy(BitConverter.GetBytes((ushort)719), 0, p, 18, 2);
            Array.Copy(BitConverter.GetBytes((ushort)10), 0, p, 20, 2); Array.Copy(BitConverter.GetBytes((uint)10001), 0, p, 24, 4);
            BoardStatus s = Protocol.ReadStatus(Decode(Protocol.Encode(4, 0x82, p)));
            Check(s.FrameId == 0x12345678 && s.SampleId == 43 && s.Uptime == 10001 && s.X == 1279 && s.Y == 719, "status LE offsets");
            Check(s.CoordinateUsable(239) && !s.CoordinateUsable(240), "result age includes time on host");
            s.TransportAge = 1000; Check(!s.CoordinateUsable(0), "cached response after retry cannot revive old coordinates"); s.TransportAge = 0;
            s.Flags |= 16; Check(!s.CoordinateUsable(0), "pending clears point"); s.Flags = 15;
            s.X = 1280; Check(!s.CoordinateUsable(0), "out of range clears point"); s.X = 500;
            s.Flags = 13; Check(!s.CoordinateUsable(0), "invalid result clears point"); s.Flags = 14; Check(!s.CoordinateUsable(0), "video stopped clears point");
            s.Flags = 15; s.Algorithm = 2; Check(!s.CoordinateUsable(0), "non-circle clears point");
            s.Algorithm = 5; s.X = 65535; s.Y = 65535; Check(!s.CoordinateUsable(0), "sentinel never plotted");
            s.X = 100; s.Y = 100; Check(!s.CoordinateUsable(1500), "stale host snapshot never plotted");
            using (DemoTransport demo = new DemoTransport())
            {
                Exchange client = new Exchange(demo, delegate { return false; }, delegate(string x) { }, 0, delegate(byte b) { });
                Check(Protocol.ReadPing(client.Request(1, new byte[0], false, 50)).Capabilities == 59, "demo PING codec");
                for (byte id = 0; id < 6; id++)
                {
                    Frame ack = client.Request(16, new byte[] { id }, false, 50); Check(ack.Payload[1] == id, "demo ACK " + id);
                    Check(Protocol.ReadStatus(client.Request(2, new byte[0], false, 50)).Algorithm == id, "demo queried mode " + id);
                }
            }
        }
        static void LifecycleTests()
        {
            SerialController controller = new SerialController("unused", true);
            ManualResetEvent ended = new ManualResetEvent(false); bool connected = false, changed = false, brightChanged = false; int statuses = 0;
            controller.Connected = delegate(BoardInfo i) { connected = true; controller.Send(16, 5); };
            controller.AlgorithmCompleted = delegate(byte id, bool ok, string msg) { changed = ok && id == 5; if (changed) controller.Send(Protocol.SetBrightness, 224); };
            controller.BrightnessCompleted = delegate(int value, bool ok, string msg) { brightChanged = ok && value == -32; };
            controller.Status = delegate(BoardStatus s) { if (s.Algorithm == 5) Interlocked.Increment(ref statuses); };
            controller.Ended = delegate(string s) { ended.Set(); };
            controller.Start(); Thread.Sleep(650); controller.Dispose();
            Check(ended.WaitOne(1500), "worker stops and closes transport"); controller.Dispose(); controller.Send(2, 0);
            Check(connected && changed && brightChanged && statuses > 0, "background handshake, algorithm/brightness commands, status and shutdown"); ended.Dispose();
        }
        static void BrightnessTests()
        {
            Check(Protocol.Hex(Protocol.Encode(1, Protocol.SetBrightness, Protocol.BrightnessPayload(32))) == "AA 55 01 01 11 01 20 E9 AE", "frozen brightness +32 packet");
            Check(Protocol.Hex(Protocol.Encode(2, Protocol.SetBrightness, Protocol.BrightnessPayload(-32))) == "AA 55 01 02 11 01 E0 79 EC", "frozen brightness -32 packet");
            Check(Protocol.Hex(Protocol.Encode(4, Protocol.GetBrightness, new byte[0])) == "AA 55 01 04 12 00 A5 4B", "frozen brightness query");
            BoardInfo oldInfo = Protocol.ReadPing(Decode(Protocol.Encode(1, 0x81, new byte[] { 0, 1, 11, 0, 0, 5, 208, 2 })));
            Check(!oldInfo.SupportsBrightness, "legacy firmware keeps normal controls but has no brightness capability");
            using (DemoTransport demo = new DemoTransport())
            {
                Exchange exchange = new Exchange(demo, delegate { return false; }, delegate(string s) { }, 0, delegate(byte b) { });
                Check(Protocol.ReadPing(exchange.Request(1, new byte[0], false, 50)).SupportsBrightness, "brightness capability advertised");
                Check(Protocol.ReadBrightness(exchange.Request(Protocol.GetBrightness, new byte[0], false, 50)) == 0, "brightness defaults to zero");
                for (int value = -128; value <= 127; value++)
                {
                    Frame reply = exchange.Request(Protocol.SetBrightness, Protocol.BrightnessPayload(value), false, 50);
                    Check(Protocol.ReadBrightness(reply) == value, "signed brightness round trip " + value);
                }
                BoardStatus status = Protocol.ReadStatus(exchange.Request(2, new byte[0], false, 50));
                Check(status.Algorithm == 0 && !status.UartOwner && status.StreamPeriod == 0, "brightness leaves mode/ownership/legacy status unchanged");
                Check(Protocol.ReadBrightness(exchange.Request(Protocol.GetBrightness, new byte[0], false, 50)) == 127, "brightness query returns confirmed value");
                exchange.Request(Protocol.SetBrightness, Protocol.BrightnessPayload(0), false, 50);
                Check(Protocol.ReadBrightness(exchange.Request(Protocol.GetBrightness, new byte[0], false, 50)) == 0, "brightness restore default");
            }
            foreach (int invalid in new int[] { -129, 128 })
            {
                bool rejected = false; try { Protocol.BrightnessPayload(invalid); } catch (ArgumentOutOfRangeException) { rejected = true; }
                Check(rejected, "brightness range rejected " + invalid);
            }
            bool badReply = false;
            try { Protocol.ReadBrightness(Decode(Protocol.Encode(1, 0x92, new byte[] { 0 }))); }
            catch (InvalidOperationException) { badReply = true; }
            Check(badReply, "short brightness response rejected");
        }
        static void ImageAutoTests()
        {
            Check(!new BoardInfo { Capabilities=27 }.SupportsImageAuto,"old firmware disables image controls");
            using(DemoTransport demo=new DemoTransport())
            {
                Exchange exchange=new Exchange(demo,delegate{return false;},delegate(string x){},0,delegate(byte b){});
                Check(Protocol.ReadPing(exchange.Request(1,new byte[0],false,50)).SupportsImageAuto,"image capability advertised");
                for(byte flags=0;flags<4;flags++)
                {
                    Check(Protocol.ImageAutoPayload((flags&1)!=0,(flags&2)!=0)==flags,"image flags encoding "+flags);
                    Check(Protocol.ReadImageAutoAck(exchange.Request(Protocol.SetImageAuto,new byte[]{flags},false,50))==flags,"image ACK "+flags);
                    ImageAutoStatus image=Protocol.ReadImageAuto(exchange.Request(Protocol.GetImageAuto,new byte[0],false,50));
                    Check(image.Awb==((flags&1)!=0) && image.Ae==((flags&2)!=0) && image.MeanValid && image.SensorValid,"image readback flags "+flags);
                    Check(image.Mean==128 && image.Target==128 && image.Exposure==1600 && image.Gain==48,"image telemetry offsets");
                    Check(image.RedGain==((flags&1)!=0 ? 0x12000u : 0x10000u),"AWB unity bypass telemetry");
                }
                exchange.Request(Protocol.SetBrightness,Protocol.BrightnessPayload(32),false,50);
                Check(Protocol.ReadImageAuto(exchange.Request(Protocol.GetImageAuto,new byte[0],false,50)).Mean==128,"display brightness independent of AE statistics");
                bool rejected=false;
                try { exchange.Request(Protocol.SetImageAuto,new byte[]{4},false,50); } catch(CommandRejected e){rejected=e.Code==3;}
                Check(rejected,"invalid image flags rejected");
            }
            bool bad=false;
            try { Protocol.ReadImageAuto(new Frame { Command=0x94, Payload=new byte[15] }); } catch(InvalidOperationException){bad=true;}
            Check(bad,"short image report rejected");
        }
        static Control Find(Control root, Predicate<Control> match)
        {
            if (match(root)) return root;
            foreach (Control child in root.Controls) { Control found = Find(child, match); if (found != null) return found; }
            return null;
        }
        static bool PumpUntil(Func<bool> condition)
        {
            System.Diagnostics.Stopwatch timeout = System.Diagnostics.Stopwatch.StartNew();
            while (timeout.ElapsedMilliseconds < 2500) { Application.DoEvents(); if (condition()) return true; Thread.Sleep(10); }
            return false;
        }
        static void UiLoopTests()
        {
            Application.EnableVisualStyles(); Application.SetCompatibleTextRenderingDefault(false);
            using (MainForm form = new MainForm(true))
            {
                IntPtr hiddenHandle = form.Handle; // Message target only; Form is never shown.
                Control root = form.Controls[0]; form.Controls.Remove(root);
                using (root)
                {
                    root.Dock = DockStyle.None; root.Size = form.ClientSize; root.Visible = true; root.CreateControl(); LayoutTree(root);
                    Button connect = (Button)Find(root, delegate(Control c) { return c is Button && c.Text == "连接设备"; });
                    Button circle = (Button)Find(root, delegate(Control c) { return c is Button && c.Text == "圆心检测与跟踪"; });
                    Button query = (Button)Find(root, delegate(Control c) { return c is Button && c.Text == "立即查询"; });
                    PointView point = (PointView)Find(root, delegate(Control c) { return c is PointView; });
                    Button brightApply = (Button)Find(root, delegate(Control c) { return c is Button && c.Text == "应用亮度"; });
                    Button brightReset = (Button)Find(root, delegate(Control c) { return c is Button && c.Text == "亮度归零"; });
                    NumericUpDown brightInput = (NumericUpDown)Find(root, delegate(Control c) { return c is NumericUpDown; });
                    Check(!brightApply.Enabled && !brightInput.Enabled, "brightness disabled before handshake");
                    Check(!circle.Enabled && !query.Enabled, "UI disables commands before handshake");
                    connect.PerformClick(); Check(PumpUntil(delegate { return query.Enabled; }), "UI receives background handshake through message loop");
                    circle.PerformClick(); Check(!circle.Enabled && !point.Valid, "UI clears point and disables repeated switching while waiting");
                    Check(PumpUntil(delegate { return circle.Enabled && point.Valid; }), "UI displays confirmed circle status and point");
                    Check(PumpUntil(delegate { return brightApply.Enabled; }), "UI reads brightness after capability handshake");
                    brightInput.Value = 32; brightApply.PerformClick();
                    Check(!brightApply.Enabled && !circle.Enabled, "brightness waits for ACK and prevents conflicting commands");
                    Check(PumpUntil(delegate { return brightApply.Enabled && Find(root, delegate(Control c) { return c is Label && c.Text == "当前：+32"; }) != null; }), "UI confirms applied brightness");
                    Check(Find(root, delegate(Control c) { return c is Label && c.Text == "圆心检测与跟踪"; }) != null, "brightness does not switch selected algorithm");
                    brightReset.PerformClick();
                    Check(PumpUntil(delegate { return brightApply.Enabled && brightInput.Value == 0 && Find(root, delegate(Control c) { return c is Label && c.Text == "当前：0"; }) != null; }), "UI brightness restore default");
                    Button imageApply=(Button)Find(root,delegate(Control c){return c is Button && c.Text=="应用图像设置";});
                    CheckBox awb=(CheckBox)Find(root,delegate(Control c){return c is CheckBox && c.Text=="自动白平衡 AWB";});
                    CheckBox ae=(CheckBox)Find(root,delegate(Control c){return c is CheckBox && c.Text=="自动曝光 AE";});
                    Check(PumpUntil(delegate{return imageApply.Enabled;}),"UI reads image settings");
                    awb.Checked=false;ae.Checked=false;imageApply.PerformClick();
                    Check(!imageApply.Enabled && !brightApply.Enabled && !circle.Enabled,"image transaction blocks conflicting controls");
                    Check(PumpUntil(delegate{return imageApply.Enabled && Find(root,delegate(Control c){return c is Label && c.Text.Contains("AWB关 / AE关");})!=null;}),"UI confirms AWB/AE OFF");
                    awb.Checked=true;ae.Checked=true;imageApply.PerformClick();
                    Check(PumpUntil(delegate{return imageApply.Enabled && Find(root,delegate(Control c){return c is Label && c.Text.Contains("AWB开 / AE开");})!=null;}),"UI confirms AWB/AE ON");
                    connect.PerformClick(); Check(PumpUntil(delegate { return connect.Enabled && connect.Text == "连接设备"; }) && !point.Valid && !query.Enabled, "UI disconnect clears point and controls");
                    connect.PerformClick(); Check(PumpUntil(delegate { return query.Enabled && Find(root, delegate(Control c) { return c is Label && c.Text == "原图"; }) != null; }), "UI reconnect queries actual mode without replaying old choice");
                    connect.PerformClick(); Check(PumpUntil(delegate { return connect.Text == "连接设备" && connect.Enabled; }), "UI second disconnect closes cleanly");
                }
            }
        }
        static void Vectors(string path)
        {
            List<string> lines = new List<string>(); lines.Add("PING|" + Protocol.Hex(Protocol.Encode(1, 1, new byte[0]))); lines.Add("STATUS|" + Protocol.Hex(Protocol.Encode(2, 2, new byte[0])));
            for (byte id = 0; id < 6; id++) lines.Add("ALGO" + id + "|" + Protocol.Hex(Protocol.Encode((byte)(id + 3), 16, new byte[] { id })));
            lines.Add("GRAY_ACK|" + Protocol.Hex(Protocol.Encode(5, 0x90, new byte[] { 0, 2 }))); File.WriteAllLines(path, lines, Encoding.UTF8);
            byte[] p = new byte[28]; p[1] = 5; p[2] = 255; p[3] = 15;
            ushort[] words = { 1280, 720, 812, 296, 10, 0 }; int[] offsets = { 4, 6, 16, 18, 20, 22 };
            for (int i = 0; i < words.Length; i++) Array.Copy(BitConverter.GetBytes(words[i]), 0, p, offsets[i], 2);
            Array.Copy(BitConverter.GetBytes((uint)14832), 0, p, 8, 4); Array.Copy(BitConverter.GetBytes((uint)127), 0, p, 12, 4); Array.Copy(BitConverter.GetBytes((uint)247200), 0, p, 24, 4);
            File.AppendAllText(path, "STATUS_REPLY|" + Protocol.Hex(Protocol.Encode(10, 0x82, p)) + Environment.NewLine, Encoding.UTF8);
        }
        static void Preview(string path, Size size)
        {
            Application.EnableVisualStyles(); Application.SetCompatibleTextRenderingDefault(false);
            using (MainForm form = new MainForm(true))
            {
                form.WindowState = FormWindowState.Normal;
                form.ClientSize = size;
                // Detach the client panel to create child handles without showing a Form.
                // No Application.Run, Show, top-level window or serial connection.
                form.PerformLayout(); form.PreparePreview();
                Control root = form.Controls[0]; form.Controls.Remove(root);
                using (root)
                {
                    root.Dock = DockStyle.None; root.Size = form.ClientSize; root.Visible = true; root.CreateControl(); LayoutTree(root);
                    CheckCombos(root);
                    using (Bitmap image = new Bitmap(root.Width, root.Height)) { root.DrawToBitmap(image, new Rectangle(Point.Empty, root.Size)); image.Save(path, System.Drawing.Imaging.ImageFormat.Png); }
                }
            }
        }
        static void LayoutTree(Control control) { control.PerformLayout(); foreach (Control child in control.Controls) { child.CreateControl(); LayoutTree(child); } }
        static void CheckCombos(Control control)
        {
            ComboBox combo = control as ComboBox;
            if (combo != null && combo.Items.Count == 3 && combo.Items[0].ToString() == "200 ms") Check(combo.SelectedIndex == 0 && combo.Text == "200 ms", "default polling selection");
            foreach (Control child in control.Controls) CheckCombos(child);
        }
        [STAThread]
        public static int Main(string[] args)
        {
            try
            {
                if (args.Length > 0 && args[0] == "--preview")
                {
                    Size size = args.Length >= 4 ? new Size(int.Parse(args[2]), int.Parse(args[3])) : args.Length > 2 ? new Size(1008, 701) : new Size(1920, 1000);
                    Preview(args[1], size); Console.WriteLine("PASS: offline UI preview " + size.Width + "x" + size.Height); return 0;
                }
                CodecTests(); ExchangeTests(); StatusTests(); BrightnessTests(); ImageAutoTests(); LifecycleTests(); UiLoopTests();
                if (args.Length > 1 && args[0] == "--vectors") Vectors(args[1]);
                Console.WriteLine("PASS: " + checks + " offline checks; no COM port opened."); return 0;
            }
            catch (Exception e) { Console.Error.WriteLine(e.ToString()); return 1; }
        }
    }
}
