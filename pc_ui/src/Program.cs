using System;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace FpgaControl
{
    static class Program
    {
        [DllImport("user32.dll")] static extern bool SetProcessDPIAware();
        [STAThread]
        static void Main(string[] args)
        {
            try
            {
                SetProcessDPIAware(); Application.EnableVisualStyles(); Application.SetCompatibleTextRenderingDefault(false);
                Application.Run(new MainForm(Array.IndexOf(args, "--demo") >= 0));
            }
            catch (Exception error) { MessageBox.Show(error.ToString(), "启动失败", MessageBoxButtons.OK, MessageBoxIcon.Error); }
        }
    }
}
