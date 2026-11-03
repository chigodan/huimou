using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;
using System.Windows.Forms;

namespace FpgaControl
{
    public sealed class MainForm : Form
    {
        static readonly Color Ink = Color.FromArgb(26, 43, 66), Muted = Color.FromArgb(98, 113, 133), Blue = Color.FromArgb(34, 104, 213), Green = Color.FromArgb(17, 133, 105);
        readonly bool demo;
        readonly ComboBox ports = new ComboBox(), rate = new ComboBox();
        readonly Button connect = MakeButton("连接设备", true), refresh = MakeButton("刷新串口", false), ping = MakeButton("检查连接", false), query = MakeButton("立即查询", false);
        readonly CheckBox verbose = new CheckBox();
        readonly TrackBar brightnessSlider = new TrackBar();
        readonly NumericUpDown brightnessInput = new NumericUpDown();
        readonly Button brightnessApply = MakeButton("应用亮度", true), brightnessReset = MakeButton("亮度归零", false);
        readonly CheckBox awbCheck=new CheckBox { Text="自动白平衡 AWB", Checked=true, AutoSize=true };
        readonly CheckBox aeCheck=new CheckBox { Text="自动曝光 AE", Checked=true, AutoSize=true };
        readonly Button imageApply=MakeButton("应用图像设置",true);
        readonly Label imageState=TextLabel("等待图像状态",10,false);
        ImageAutoStatus imageSnapshot;
        bool imageKnown,imageChanging,imageDraftDirty,imageUpdating;
        long imageReceivedMs;
        readonly Label brightnessState = TextLabel("当前：—", 10, false);
        readonly Label connection = TextLabel("未连接", 12, true), active = TextLabel("—", 21, true), video = TextLabel("等待连接", 18, true), result = TextLabel("—", 18, true);
        readonly Label owner = TextLabel("—", 10, false), tracking = TextLabel("—", 10, false), pending = TextLabel("—", 10, false), dimensions = TextLabel("—", 10, false), frames = TextLabel("—", 10, false), samples = TextLabel("—", 10, false), age = TextLabel("—", 10, false), uptime = TextLabel("—", 10, false);
        readonly Label footer = TextLabel("先连接串口，再选择算法。图像在 HDMI 显示器上查看。", 9, false);
        readonly Label coordinates = TextLabel("X  —     Y  —", 22, true), coordinateHint = TextLabel("等待圆心模式的有效结果", 10, false);
        readonly RichTextBox log = new RichTextBox();
        readonly Button[] algorithms = new Button[6];
        readonly PointView plot = new PointView();
        readonly Stopwatch clock = Stopwatch.StartNew();
        readonly System.Windows.Forms.Timer timer = new System.Windows.Forms.Timer();
        SerialController controller;
        BoardStatus latest;
        BoardInfo info;
        long receivedMs;
        bool connected, changing, closing, brightnessChanging, brightnessKnown;
        string lastStatusSummary;

        public MainForm(bool demo)
        {
            this.demo = demo;
            Text = "控制台" + (demo ? " · 离线演示" : "");
            Font = new Font("Microsoft YaHei UI", 9.5f);
            BackColor = Color.FromArgb(241, 245, 250); ForeColor = Ink;
            AutoScaleMode = AutoScaleMode.Dpi;
            ClientSize = new Size(1180, 820); MinimumSize = new Size(1024, 740); StartPosition = FormStartPosition.CenterScreen;
            WindowState = FormWindowState.Maximized;
            BuildLayout(); RefreshPorts(); UpdateControls();
            refresh.Click += delegate { RefreshPorts(); };
            connect.Click += delegate { ToggleConnection(); };
            ping.Click += delegate { if (controller != null) controller.Send(1, 0); };
            query.Click += delegate { if (controller != null) controller.Send(2, 0); };
            rate.SelectedIndexChanged += delegate { if (controller != null) controller.PollMs = PollInterval; };
            verbose.CheckedChanged += delegate { if (controller != null) controller.Verbose = verbose.Checked; };
            brightnessSlider.ValueChanged += delegate { brightnessInput.Value = brightnessSlider.Value; };
            brightnessInput.ValueChanged += delegate { brightnessSlider.Value = (int)brightnessInput.Value; };
            awbCheck.CheckedChanged += delegate { if(!imageUpdating) imageDraftDirty=true; };
            aeCheck.CheckedChanged += delegate { if(!imageUpdating) imageDraftDirty=true; };
            imageApply.Click += delegate { ApplyImageAuto(); };
            brightnessApply.Click += delegate { ApplyBrightness((int)brightnessInput.Value); };
            brightnessReset.Click += delegate { SetBrightnessControls(0); ApplyBrightness(0); };
            timer.Interval = 50; timer.Tick += delegate { RenderStatus(); }; timer.Start();
            FormClosing += delegate { closing = true; timer.Stop(); timer.Dispose(); if (controller != null) controller.Dispose(); };
            AddLog(demo ? "离线演示模式：所有状态和坐标为模拟数据，不打开串口。" : "请连接板载 USB 并关闭串口助手。图像自动调节需要新固件；显示亮度仅作用于显示端。");
            if (demo) Shown += delegate { ToggleConnection(); };
        }
        int PollInterval { get { return new int[] { 200, 500, 1000 }[Math.Max(0, rate.SelectedIndex)]; } }
        static Label TextLabel(string value, float size, bool bold)
        {
            return new Label { Text = value, Font = new Font("Microsoft YaHei UI", size, bold ? FontStyle.Bold : FontStyle.Regular), ForeColor = bold ? Ink : Muted, AutoSize = false, Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft, AutoEllipsis = true, Margin = new Padding(0) };
        }
        static Button MakeButton(string value, bool primary)
        {
            Button b = new Button { Text = value, FlatStyle = FlatStyle.Flat, BackColor = primary ? Blue : Color.White, ForeColor = primary ? Color.White : Ink, Height = 36, Width = 110, Margin = new Padding(0, 0, 10, 0), Cursor = Cursors.Hand, UseVisualStyleBackColor = false };
            b.FlatAppearance.BorderColor = primary ? Blue : Color.FromArgb(216, 225, 237); b.FlatAppearance.BorderSize = 1;
            return b;
        }
        static Panel Card() { return new Panel { BackColor = Color.White, Dock = DockStyle.Fill, Padding = new Padding(16), Margin = new Padding(0, 0, 12, 12) }; }
        static TableLayoutPanel Table(int columns, int rows)
        {
            TableLayoutPanel table = new TableLayoutPanel { ColumnCount = columns, RowCount = rows, Dock = DockStyle.Fill, Margin = new Padding(0), Padding = new Padding(0) };
            if (rows == 1) table.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            return table;
        }
        void BuildLayout()
        {
            Panel viewport = new Panel { Dock = DockStyle.Fill, BackColor = BackColor };
            Controls.Add(viewport);
            TableLayoutPanel root = Table(1, 7); root.Padding = new Padding(20, 12, 8, 8);
            viewport.Controls.Add(root);
            for (int row = 0; row < 7; row++) root.RowStyles.Add(new RowStyle(row == 4 ? SizeType.Percent : SizeType.Absolute, row == 4 ? 100 : 0));
            TableLayoutPanel header = Table(2, 1); header.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 70)); header.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30));
            Label title = TextLabel("控制台", 22, true); title.MinimumSize = new Size(0, 72); header.Controls.Add(title, 0, 0);
            Label badge = TextLabel(demo ? "●  离线演示 / 模拟数据" : "●  设备尚未连接", 11, true); badge.Name = "badge"; badge.ForeColor = demo ? Color.DarkOrange : Muted; badge.TextAlign = ContentAlignment.MiddleRight; header.Controls.Add(badge, 1, 0); root.Controls.Add(header, 0, 0);

            Panel connectionCard = Card(); connectionCard.Padding = new Padding(16, 0, 16, 0); root.Controls.Add(connectionCard, 0, 1);
            TableLayoutPanel controls = Table(8, 1); connectionCard.Controls.Add(controls);
            foreach (float width in new float[] { 43 * 1.2f, 105 * 1.2f, 120 * 1.2f, 120 * 1.2f, 119 * 1.4f }) controls.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, width));
            controls.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100)); controls.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 43 * 1.2f)); controls.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 100 * 1.2f));
            controls.Controls.Add(TextLabel("串口", 10, true), 0, 0);
            ports.Dock = DockStyle.Fill; ports.Margin = new Padding(0, 0, 10, 0); controls.Controls.Add(ports, 1, 0);
            refresh.Dock = connect.Dock = DockStyle.Fill; controls.Controls.Add(refresh, 2, 0); controls.Controls.Add(connect, 3, 0);
            controls.Controls.Add(TextLabel("115200 · 8N1", 9, false), 4, 0); controls.Controls.Add(connection, 5, 0);
            controls.Controls.Add(TextLabel("刷新", 9, false), 6, 0);
            rate.DropDownStyle = ComboBoxStyle.DropDownList; rate.Items.AddRange(new object[] { "200 ms", "500 ms", "1000 ms" }); rate.SelectedIndex = 0; rate.Dock = DockStyle.Fill; rate.Margin = new Padding(0); controls.Controls.Add(rate, 7, 0);

            TableLayoutPanel metrics = Table(3, 1); for (int c = 0; c < 3; c++) metrics.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.333f));
            metrics.Controls.Add(Metric("当前算法", active), 0, 0); metrics.Controls.Add(Metric("视频输出帧", video), 1, 0); metrics.Controls.Add(Metric("圆心检测结果", result), 2, 0); root.Controls.Add(metrics, 0, 2);
            Panel brightnessCard = Card(); brightnessCard.Padding = new Padding(16, 4, 16, 4); root.Controls.Add(brightnessCard, 0, 3);
            TableLayoutPanel imageCard=Table(1,2);
            imageCard.RowStyles.Add(new RowStyle(SizeType.Absolute,42));
            imageCard.RowStyles.Add(new RowStyle(SizeType.Percent,100));brightnessCard.Controls.Add(imageCard);
            TableLayoutPanel imageTable=Table(5,1);imageCard.Controls.Add(imageTable,0,0);
            // Let text determine its width, including Windows font/DPI scaling.
            for(int c=0;c<4;c++) imageTable.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            imageTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100));
            Label imageTitle=TextLabel("图像自动调节",10,true);
            imageTitle.AutoSize=true;imageTitle.AutoEllipsis=false;
            imageTitle.Dock=DockStyle.None;imageTitle.Anchor=AnchorStyles.Left;imageTitle.Margin=new Padding(0,0,16,0);
            imageTable.Controls.Add(imageTitle,0,0);
            awbCheck.Font=aeCheck.Font=new Font("Microsoft YaHei UI",10);
            awbCheck.Dock=aeCheck.Dock=DockStyle.None;awbCheck.Anchor=aeCheck.Anchor=AnchorStyles.Left;
            awbCheck.TextAlign=aeCheck.TextAlign=ContentAlignment.MiddleLeft;
            awbCheck.CheckAlign=aeCheck.CheckAlign=ContentAlignment.MiddleLeft;
            awbCheck.Margin=aeCheck.Margin=new Padding(0,0,20,0);
            imageTable.Controls.Add(awbCheck,1,0);imageTable.Controls.Add(aeCheck,2,0);
            imageApply.Font=new Font("Microsoft YaHei UI",10);
            imageApply.AutoSize=true;imageApply.AutoSizeMode=AutoSizeMode.GrowAndShrink;
            imageApply.MinimumSize=new Size(130,36);imageApply.Padding=new Padding(12,0,12,0);
            imageApply.Dock=DockStyle.Fill;imageApply.Margin=new Padding(0,3,12,3);imageTable.Controls.Add(imageApply,3,0);
            imageTable.Controls.Add(imageState,4,0);
            TableLayoutPanel brightnessTable = Table(6, 1); imageCard.Controls.Add(brightnessTable,0,1);
            brightnessTable.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 90));
            brightnessTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            foreach (float width in new float[] { 76, 110, 110, 145 }) brightnessTable.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, width));
            brightnessTable.Controls.Add(TextLabel("显示亮度", 10, true), 0, 0);
            brightnessSlider.Minimum = -128; brightnessSlider.Maximum = 127; brightnessSlider.TickFrequency = 32;
            brightnessSlider.SmallChange = 1; brightnessSlider.LargeChange = 16; brightnessSlider.AutoSize = false;
            brightnessSlider.Dock = DockStyle.Fill; brightnessSlider.Margin = new Padding(0, 4, 10, 0);
            brightnessTable.Controls.Add(brightnessSlider, 1, 0);
            brightnessInput.Minimum = -128; brightnessInput.Maximum = 127; brightnessInput.Dock = DockStyle.Fill;
            brightnessInput.Margin = new Padding(0, 9, 10, 0); brightnessTable.Controls.Add(brightnessInput, 2, 0);
            brightnessApply.Dock = brightnessReset.Dock = DockStyle.Fill;
            brightnessApply.Margin = brightnessReset.Margin = new Padding(0, 4, 10, 4);
            brightnessTable.Controls.Add(brightnessApply, 3, 0); brightnessTable.Controls.Add(brightnessReset, 4, 0);
            brightnessTable.Controls.Add(brightnessState, 5, 0);

            TableLayoutPanel body = Table(2, 1); body.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 46)); body.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 54)); root.Controls.Add(body, 0, 4);
            TableLayoutPanel left = Table(1, 2); left.RowStyles.Add(new RowStyle(SizeType.Percent, 48)); left.RowStyles.Add(new RowStyle(SizeType.Percent, 52)); body.Controls.Add(left, 0, 0);
            Panel algoCard = Card(); algoCard.Padding = new Padding(16, 6, 16, 6); left.Controls.Add(algoCard, 0, 0);
            TableLayoutPanel algoTable = Table(3, 3); for (int c = 0; c < 3; c++) algoTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.333f));
            algoTable.RowStyles.Add(new RowStyle(SizeType.Absolute, 24)); algoTable.RowStyles.Add(new RowStyle(SizeType.Percent, 50)); algoTable.RowStyles.Add(new RowStyle(SizeType.Percent, 50)); algoCard.Controls.Add(algoTable);
            Label algoHeading = TextLabel("算法选择  /  等待 FPGA 确认后生效", 10, true); algoTable.Controls.Add(algoHeading, 0, 0); algoTable.SetColumnSpan(algoHeading, 3);
            for (int a = 0; a < 6; a++)
            {
                int id = a; Button b = MakeButton(Protocol.Algorithms[a], false); b.Dock = DockStyle.Fill; b.Margin = new Padding(0, 0, 8, 3); algorithms[a] = b; algoTable.Controls.Add(b, a % 3, 1 + a / 3);
                b.Click += delegate { SelectAlgorithm((byte)id); };
            }
            Panel statusCard = Card(); statusCard.Padding = new Padding(16, 6, 16, 6); left.Controls.Add(statusCard, 0, 1); TableLayoutPanel detail = Table(2, 5); statusCard.Controls.Add(detail);
            detail.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50)); detail.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            detail.RowStyles.Add(new RowStyle(SizeType.Absolute, 20)); for (int r = 0; r < 4; r++) detail.RowStyles.Add(new RowStyle(SizeType.Percent, 25));
            Label stateTitle = TextLabel("设备状态", 10, true); detail.Controls.Add(stateTitle, 0, 0); detail.SetColumnSpan(stateTitle, 2);
            Label[] details = { owner, tracking, pending, dimensions, frames, samples, age, uptime };
            for (int d = 0; d < details.Length; d++) detail.Controls.Add(details[d], d % 2, 1 + d / 2);

            Panel pointCard = Card(); pointCard.Padding = new Padding(16, 12, 16, 12); body.Controls.Add(pointCard, 1, 0); TableLayoutPanel pointTable = Table(1, 5); pointCard.Controls.Add(pointTable);
            pointTable.RowStyles.Add(new RowStyle(SizeType.Absolute, 26)); pointTable.RowStyles.Add(new RowStyle(SizeType.Absolute, 40)); pointTable.RowStyles.Add(new RowStyle(SizeType.Absolute, 24)); pointTable.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); pointTable.RowStyles.Add(new RowStyle(SizeType.Absolute, 20));
            pointTable.Controls.Add(TextLabel("圆心坐标  /  图像坐标示意", 11, true), 0, 0); pointTable.Controls.Add(coordinates, 0, 1); pointTable.Controls.Add(coordinateHint, 0, 2);
            plot.Dock = DockStyle.Fill; plot.Margin = new Padding(0, 6, 0, 4); pointTable.Controls.Add(plot, 0, 3); pointTable.Controls.Add(TextLabel("左上角为原点，X 向右、Y 向下；示意图不包含视频。", 9, false), 0, 4);

            Panel logCard = Card(); logCard.Padding = new Padding(16, 10, 16, 10); root.Controls.Add(logCard, 0, 5); TableLayoutPanel logTable = Table(1, 2); logTable.RowStyles.Add(new RowStyle(SizeType.AutoSize)); logTable.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); logCard.Controls.Add(logTable);
            FlowLayoutPanel tools = new FlowLayoutPanel { Dock = DockStyle.Fill, WrapContents = true, AutoSize = true, AutoSizeMode = AutoSizeMode.GrowAndShrink, Margin = new Padding(0) };
            Label logTitle = TextLabel("通信日志", 10, true); logTitle.Dock = DockStyle.None; logTitle.AutoSize = true; logTitle.MinimumSize = new Size(90, 36); logTitle.Margin = new Padding(0, 0, 8, 4); tools.Controls.Add(logTitle);
            tools.Controls.Add(ping); tools.Controls.Add(query);
            Button clear = MakeButton("清空", false), export = MakeButton("保存日志", false); tools.Controls.Add(clear); tools.Controls.Add(export);
            foreach (Button button in new Button[] { ping, query, clear, export })
            {
                button.AutoSize = true; button.AutoSizeMode = AutoSizeMode.GrowAndShrink;
                button.MinimumSize = new Size(button == clear ? 65 : 110, 36);
                button.Padding = new Padding(10, 4, 10, 4); button.Margin = new Padding(0, 0, 8, 4);
            }
            verbose.Text = "显示轮询报文"; verbose.AutoSize = true; verbose.Margin = new Padding(8, 8, 0, 4); tools.Controls.Add(verbose); logTable.Controls.Add(tools, 0, 0);
            clear.Click += delegate { log.Clear(); }; export.Click += delegate { ExportLog(); };
            log.Dock = DockStyle.Fill; log.ReadOnly = true; log.BorderStyle = BorderStyle.None; log.BackColor = Color.FromArgb(246, 248, 251); log.ForeColor = Ink; log.Font = new Font("Consolas", 9); log.DetectUrls = false; logTable.Controls.Add(log, 0, 1);
            root.Controls.Add(footer, 0, 6);
            root.SizeChanged += delegate { ResizeSections(root, title, tools, logCard, connectionCard); };
            ResizeSections(root, title, tools, logCard, connectionCard);
        }
        static void ResizeSections(TableLayoutPanel root, Label title, FlowLayoutPanel tools, Panel logCard, Panel connectionCard)
        {
            // Keep readable controls on smaller screens; distribute the rest with the window.
            float height = Math.Max(0, root.ClientSize.Height - root.Padding.Vertical);
            root.SuspendLayout();
            bool compact=height<780;
            title.MinimumSize=new Size(0,compact ? 56 : 72);
            logCard.Padding=new Padding(16,compact ? 3 : 10,16,compact ? 3 : 10);
            root.RowStyles[0].Height = Math.Max(Math.Max(title.MinimumSize.Height, title.Font.Height + 16), height * 0.09f);
            float previousConnectionCardHeight = (Math.Max(64, height * 0.09f) - connectionCard.Margin.Vertical) * 0.7f;
            root.RowStyles[1].Height = previousConnectionCardHeight * 0.7f + connectionCard.Margin.Vertical;
            root.RowStyles[2].Height = Math.Max(compact ? 88 : 96, height * 0.12f);
            int toolbarWidth = Math.Max(1, root.ClientSize.Width - root.Padding.Horizontal - logCard.Margin.Horizontal - logCard.Padding.Horizontal);
            int toolbarHeight = tools.GetPreferredSize(new Size(toolbarWidth, 0)).Height;
            int logMinimum = toolbarHeight + logCard.Padding.Vertical + logCard.Margin.Vertical + 18;
            root.RowStyles[3].Height = 112;
            root.RowStyles[5].Height = Math.Max(logMinimum, height * (compact ? 0.10f : 0.16f));
            root.RowStyles[6].Height = Math.Max(24, height * 0.03f);
            root.ResumeLayout(true);
        }
        static Panel Metric(string title, Label value)
        {
            Panel panel = Card(); panel.Padding = new Padding(16, 10, 16, 10); TableLayoutPanel t = Table(1, 2); t.RowStyles.Add(new RowStyle(SizeType.Absolute, 25)); t.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); panel.Controls.Add(t); t.Controls.Add(TextLabel(title, 10, false), 0, 0); t.Controls.Add(value, 0, 1); return panel;
        }
        void RefreshPorts()
        {
            string previous = ports.Text; ports.Items.Clear();
            try { string[] names = SerialPort.GetPortNames(); Array.Sort(names); ports.Items.AddRange(names); }
            catch (Exception e) { AddLog("读取串口列表失败，可手动输入 COM 口：" + e.Message); }
            ports.Text = demo ? "模拟设备" : previous.Length > 0 ? previous : "COM5";
        }
        void Post(SerialController sender, Action action)
        {
            if (closing || IsDisposed || !IsHandleCreated) return;
            try { BeginInvoke((MethodInvoker)delegate { if (!closing && controller == sender) action(); }); }
            catch (InvalidOperationException) { }
        }
        void ToggleConnection()
        {
            if (controller != null)
            {
                connected = false; imageKnown=imageChanging=imageDraftDirty=false;imageSnapshot=null; changing = brightnessChanging = brightnessKnown = false; latest = null; connection.Text = "正在断开…"; controller.Dispose(); connect.Enabled = false; UpdateControls(); RenderStatus(); return;
            }
            string name = ports.Text.Trim().ToUpperInvariant();
            if (!demo && !Regex.IsMatch(name, "^COM[0-9]+$")) { AddLog("请填写有效串口名称，例如 COM5。"); return; }
            imageKnown=imageChanging=imageDraftDirty=false;imageSnapshot=null;
            SerialController client = new SerialController(name, demo); controller = client; latest = null; info = null; brightnessKnown = brightnessChanging = false; lastStatusSummary = null;
            client.PollMs = PollInterval; client.Verbose = verbose.Checked;
            client.Log = delegate(string message) { Post(client, delegate { AddLog(message); }); };
            client.Connected = delegate(BoardInfo board) { Post(client, delegate {
                info = board; connected = true; connection.Text = demo ? "演示已连接" : name + " 已连接"; connection.ForeColor = Green;
                AddLog(string.Format("PING 通过：协议 v1，能力 0x{0:X4}，图像 {1}×{2}。", board.Capabilities, board.Width, board.Height));
                plot.ImageWidth = board.Width; plot.ImageHeight = board.Height; UpdateControls();
            }); };
            client.Status = delegate(BoardStatus snapshot) { long arrived = clock.ElapsedMilliseconds; Post(client, delegate {
                latest = snapshot; receivedMs = arrived;
                string summary = snapshot.Algorithm + "/" + snapshot.Flags;
                if (summary != lastStatusSummary) { AddLog("设备状态：" + Protocol.AlgorithmName(snapshot.Algorithm) + "，视频" + (snapshot.VideoAlive ? "正常" : "无帧") + "，控制权" + (snapshot.UartOwner ? "UART" : "板上开关") + "。"); lastStatusSummary = summary; }
                RenderStatus();
            }); };
            client.AlgorithmCompleted = delegate(byte id, bool success, string message) { Post(client, delegate {
                changing = false; latest = null; footer.Text = success ? "FPGA 已确认：" + Protocol.AlgorithmName(id) + "。正在读取最新状态…" : "切换失败：" + message;
                UpdateControls(); RenderStatus();
            }); };
            client.BrightnessChanged = delegate(int value) { Post(client, delegate {
                brightnessState.Text = "当前：" + value.ToString("+0;-0;0");
                if (!brightnessKnown) SetBrightnessControls(value);
                brightnessKnown = true; UpdateControls();
            }); };
            client.BrightnessCompleted = delegate(int value, bool success, string message) { Post(client, delegate {
                brightnessChanging = false;
                if (success) { SetBrightnessControls(value); brightnessState.Text = "当前：" + value.ToString("+0;-0;0"); }
                footer.Text = success ? "显示亮度已生效；识别、边缘和填充的算法输入不变。" : "亮度设置失败：" + message;
                UpdateControls();
            }); };
            client.ImageAutoChanged=delegate(ImageAutoStatus snapshot) { long arrived=clock.ElapsedMilliseconds;Post(client,delegate {
                imageSnapshot=snapshot;imageReceivedMs=arrived;
                if(!imageKnown || (!imageDraftDirty && !imageChanging)) SetImageChecks(snapshot.Flags);
                imageKnown=true;UpdateControls();RenderStatus();
            }); };
            client.ImageAutoCompleted=delegate(byte flags,bool success,string message) { Post(client,delegate {
                imageChanging=false;
                if(success) { imageDraftDirty=false;SetImageChecks(flags); }
                footer.Text=success ? "图像设置已生效。AE关闭时保持当前曝光；AWB关闭时使用单位增益。" : "图像设置失败："+message;
                UpdateControls();
            }); };
            client.Ended = delegate(string reason) { Post(client, delegate {
                controller = null;imageKnown=imageChanging=imageDraftDirty=false;imageSnapshot=null; connected = changing = brightnessChanging = brightnessKnown = false; latest = null; info = null; connection.Text = "未连接"; connection.ForeColor = Muted; connect.Enabled = true;
                AddLog("连接结束：" + reason); footer.Text = reason; UpdateControls(); RenderStatus();
            }); };
            connection.Text = "正在握手…"; connection.ForeColor = Muted; UpdateControls(); AddLog(demo ? "启动模拟设备。" : "打开 " + name + "，115200 8N1；正在验证协议。"); client.Start();
        }
        void SelectAlgorithm(byte id)
        {
            if (!connected || changing || brightnessChanging || imageChanging || controller == null) return;
            changing = true; latest = null; footer.Text = "正在切换到「" + Protocol.AlgorithmName(id) + "」，等待 FPGA 帧边界确认…";
            UpdateControls(); RenderStatus(); controller.Send(16, id);
        }
        void SetBrightnessControls(int value)
        {
            brightnessInput.Value = value; brightnessSlider.Value = value;
        }
        void ApplyBrightness(int value)
        {
            if (!connected || changing || brightnessChanging || imageChanging || !brightnessKnown || controller == null || info == null || !info.SupportsBrightness) return;
            brightnessChanging = true; footer.Text = "正在应用显示亮度，等待 FPGA 帧边界确认…";
            UpdateControls(); controller.Send(Protocol.SetBrightness, Protocol.BrightnessPayload(value)[0]);
        }
        void SetImageChecks(byte flags)
        {
            imageUpdating=true;awbCheck.Checked=(flags&1)!=0;aeCheck.Checked=(flags&2)!=0;imageUpdating=false;
        }
        void ApplyImageAuto()
        {
            if(!connected || changing || brightnessChanging || imageChanging || !imageKnown || controller==null || info==null || !info.SupportsImageAuto) return;
            imageChanging=true;footer.Text="正在应用图像设置，等待视频与曝光控制域确认…";
            UpdateControls();controller.Send(Protocol.SetImageAuto,Protocol.ImageAutoPayload(awbCheck.Checked,aeCheck.Checked));
        }
        void UpdateControls()
        {
            int statusTextWidth = TextRenderer.MeasureText(connection.Text, connection.Font, Size.Empty, TextFormatFlags.SingleLine | TextFormatFlags.NoPadding).Width;
            connection.MinimumSize = new Size((int)Math.Ceiling(statusTextWidth * 1.2), 0);
            bool idle = controller == null;
            ports.Enabled = refresh.Enabled = idle && !demo;
            connect.Text = idle ? "连接设备" : "断开连接";
            ping.Enabled = query.Enabled = connected && !changing && !brightnessChanging && !imageChanging;
            for (int i = 0; i < 6; i++) algorithms[i].Enabled = connected && !changing && !brightnessChanging && !imageChanging && info != null && (info.Capabilities & 1) != 0;
            bool brightnessSupported = connected && info != null && info.SupportsBrightness;
            brightnessSlider.Enabled = brightnessInput.Enabled = brightnessApply.Enabled = brightnessReset.Enabled =
                brightnessSupported && brightnessKnown && !changing && !brightnessChanging && !imageChanging;
            bool imageSupported=connected && info!=null && info.SupportsImageAuto;
            awbCheck.Enabled=aeCheck.Enabled=imageApply.Enabled=imageSupported && imageKnown && !changing && !brightnessChanging && !imageChanging;
            if(!connected) imageState.Text="等待图像状态";
            else if(!imageSupported) imageState.Text="固件不支持图像自动调节";
            else if(imageChanging) imageState.Text="等待视频域确认…";
            else if(!imageKnown) imageState.Text="正在读取图像状态…";
            else if(imageSnapshot!=null)
                imageState.Text="当前：AWB"+(imageSnapshot.Awb ? "开" : "关")+" / AE"+(imageSnapshot.Ae ? "开" : "关")+" · "+imageSnapshot.StateText;
            if (!connected) brightnessState.Text = "当前：—";
            else if (!brightnessSupported) brightnessState.Text = "固件不支持亮度";
            else if (brightnessChanging) brightnessState.Text = "等待帧边界…";
            else if (!brightnessKnown) brightnessState.Text = "正在读取…";
            Control[] badges = Controls.Find("badge", true);
            if (badges.Length != 0) { ((Label)badges[0]).Text = demo ? "●  离线演示 / 模拟数据" : connected ? "●  UART 已连接" : "●  设备尚未连接"; badges[0].ForeColor = demo ? Color.DarkOrange : connected ? Green : Muted; }
        }
        void RenderStatus()
        {
            long elapsed = clock.ElapsedMilliseconds - receivedMs;
            bool fresh = connected && latest != null && elapsed + latest.TransportAge < 1500;
            plot.Valid = false;
            for (int i = 0; i < 6; i++) { bool selected = fresh && latest.Algorithm == i; algorithms[i].BackColor = selected ? Blue : Color.White; algorithms[i].ForeColor = selected ? Color.White : Ink; }
            if (!fresh)
            {
                active.Text = changing ? "切换中…" : "—"; video.Text = connected ? "等待新状态" : "等待连接"; video.ForeColor = Muted; result.Text = "—";
                owner.Text = "控制权：—"; tracking.Text = "跟踪使能：—"; pending.Text = "待生效：—"; dimensions.Text = "图像尺寸：—";
                frames.Text = "输出帧计数：—"; samples.Text = "结果序号：—"; age.Text = "距上次结果更新：—"; uptime.Text = "FPGA 运行时间：—";
                coordinates.Text = "X  —     Y  —"; coordinateHint.Text = connected ? "等待新的有效检测结果" : "连接后显示检测坐标";
            }
            else
            {
                BoardStatus s = latest; bool valid = !changing && s.CoordinateUsable(elapsed);
                active.Text = changing ? "切换中…" : Protocol.AlgorithmName(s.Algorithm); video.Text = s.VideoAlive ? "正常" : "未检测到视频帧"; video.ForeColor = s.VideoAlive ? Green : Color.DarkOrange;
                result.Text = valid ? "有效" : s.Algorithm == 5 ? "暂无有效结果" : "未进入圆心模式"; result.ForeColor = valid ? Green : Muted;
                owner.Text = "控制权：" + (s.UartOwner ? "UART" : "板上开关"); tracking.Text = "跟踪使能：" + (s.TrackingEnabled ? "开启" : "关闭");
                pending.Text = "待生效：" + (s.PendingAlgorithm == 255 ? "无" : Protocol.AlgorithmName(s.PendingAlgorithm)); dimensions.Text = "图像尺寸：" + s.Width + " × " + s.Height;
                frames.Text = "输出帧计数：" + s.FrameId; samples.Text = "结果序号：" + s.SampleId;
                age.Text = "距上次结果更新：" + (s.SampleAge == 65535 ? "—" : ((long)s.SampleAge + s.TransportAge + elapsed) + " ms"); uptime.Text = "FPGA 运行时间：" + TimeSpan.FromMilliseconds(s.Uptime).ToString(@"d\.hh\:mm\:ss");
                coordinates.Text = valid ? "X  " + s.X + "     Y  " + s.Y : "X  —     Y  —";
                coordinateHint.Text = valid ? "当前查询结果有效" : s.Algorithm != 5 ? "请选择「圆心检测与跟踪」" : "目标丢失、结果过期或正在切换";
                plot.ImageWidth = s.Width; plot.ImageHeight = s.Height; plot.X = s.X; plot.Y = s.Y; plot.Valid = valid;
                if (!changing && !brightnessChanging && !imageChanging) footer.Text = demo ? "离线演示：坐标与状态均为模拟数据，不代表实板运行结果。" : s.Algorithm == 5 ? "圆心模式联动双轴舵机；图像自动调节会改变摄像头画面，显示亮度仅作用于显示端。" : "状态自动更新；USB 断开后 FPGA 保持当前算法，复位后释放 UART 控制权。";
            }
            if(connected && info!=null && info.SupportsImageAuto)
            {
                bool imageFresh=imageSnapshot!=null && clock.ElapsedMilliseconds-imageReceivedMs+imageSnapshot.TransportAge<1500 && imageSnapshot.Fresh;
                frames.Text=imageFresh && imageSnapshot.MeanValid ? "测光亮度："+imageSnapshot.Mean+" / 目标 "+imageSnapshot.Target : "测光亮度：等待有效帧";
                samples.Text=imageFresh && imageSnapshot.SensorValid ? "曝光码："+imageSnapshot.Exposure+" · 增益档："+imageSnapshot.Gain : "曝光参数：等待摄像头确认";
                age.Text=imageFresh ? string.Format("白平衡：R {0:F2} / B {1:F2}",imageSnapshot.RedGain/65536.0,imageSnapshot.BlueGain/65536.0) : "白平衡增益：—";
                if(imageKnown && !imageFresh && !imageChanging) imageState.Text="图像状态已过期，等待更新";
            }
            plot.Invalidate();
        }
        void AddLog(string message)
        {
            log.AppendText(DateTime.Now.ToString("HH:mm:ss.fff") + "  " + message + Environment.NewLine);
            if (log.Lines.Length > 300)
            {
                string[] lines = log.Lines; log.Lines = new List<string>(lines).GetRange(lines.Length - 250, 250).ToArray();
            }
            log.SelectionStart = log.TextLength; log.ScrollToCaret();
        }
        void ExportLog()
        {
            using (SaveFileDialog dialog = new SaveFileDialog { Filter = "文本日志 (*.txt)|*.txt", FileName = "FPGA_UART_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".txt" })
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                try { File.WriteAllText(dialog.FileName, log.Text, new UTF8Encoding(true)); AddLog("日志已保存：" + dialog.FileName); }
                catch (Exception e) { AddLog("保存失败：" + e.Message); }
            }
        }
        public void PreparePreview()
        {
            connected = true;imageKnown=true;imageReceivedMs=clock.ElapsedMilliseconds;
            imageSnapshot=new ImageAutoStatus { Flags=31,State=2,Mean=128,Target=128,Gain=48,Exposure=1600,RedGain=0x12000,BlueGain=0xe000 };
            brightnessKnown = true; brightnessState.Text = "当前：0"; info = new BoardInfo { Capabilities = 59, Width = 1280, Height = 720 }; connection.Text = "演示预览"; connection.ForeColor = Green;
            latest = new BoardStatus { Algorithm = 5, PendingAlgorithm = 255, Flags = 15, Width = 1280, Height = 720, X = 812, Y = 296, SampleAge = 10, FrameId = 14832, SampleId = 127, Uptime = 247200 };
            receivedMs = clock.ElapsedMilliseconds; UpdateControls(); RenderStatus();
        }
        protected override void Dispose(bool disposing)
        {
            if (disposing) { closing = true; timer.Dispose(); if (controller != null) controller.Dispose(); }
            base.Dispose(disposing);
        }
    }

    public sealed class PointView : Control
    {
        public int ImageWidth = 1280, ImageHeight = 720, X, Y;
        public bool Valid;
        public PointView() { DoubleBuffered = true; BackColor = Color.FromArgb(246, 249, 253); }
        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e); Graphics g = e.Graphics; g.SmoothingMode = SmoothingMode.AntiAlias;
            if (Width < 70 || Height < 60 || ImageWidth < 1 || ImageHeight < 1) return;
            float scale = Math.Min((Width - 48f) / ImageWidth, (Height - 38f) / ImageHeight);
            RectangleF r = new RectangleF((Width - ImageWidth * scale) / 2, (Height - ImageHeight * scale) / 2, ImageWidth * scale, ImageHeight * scale);
            using (Pen grid = new Pen(Color.FromArgb(222, 231, 242)))
            using (Brush text = new SolidBrush(Color.FromArgb(108, 124, 145)))
            {
                for (int i = 0; i <= 4; i++) { g.DrawLine(grid, r.Left + r.Width * i / 4, r.Top, r.Left + r.Width * i / 4, r.Bottom); g.DrawLine(grid, r.Left, r.Top + r.Height * i / 4, r.Right, r.Top + r.Height * i / 4); }
                g.DrawString("(0, 0)", Font, text, r.Left, r.Top - 18); g.DrawString((ImageWidth - 1) + ", " + (ImageHeight - 1), Font, text, r.Right - 85, r.Bottom + 2);
                if (!Valid)
                {
                    const string message = "暂无有效圆心"; SizeF size = g.MeasureString(message, Font); g.DrawString(message, Font, text, r.Left + (r.Width - size.Width) / 2, r.Top + (r.Height - size.Height) / 2); return;
                }
            }
            float px = r.Left + X * scale, py = r.Top + Y * scale;
            using (Pen p = new Pen(Color.FromArgb(34, 104, 213), 1.5f))
            using (Brush b = new SolidBrush(Color.FromArgb(34, 104, 213)))
            {
                p.DashStyle = DashStyle.Dash; g.DrawLine(p, px, r.Top, px, r.Bottom); g.DrawLine(p, r.Left, py, r.Right, py);
                p.DashStyle = DashStyle.Solid; g.DrawEllipse(p, px - 11, py - 11, 22, 22); g.FillEllipse(b, px - 4, py - 4, 8, 8);
            }
        }
    }
}
