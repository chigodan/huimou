"""Run existing servo regressions; unrelated top-level vendor IP is omitted."""
from argparse import ArgumentParser
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import shutil
import subprocess
import sys

TESTS=['tb_servo_continuous','tb_servo_continuous_stability',
       'tb_servo_continuous_edge_start','tb_servo_continuous_calibration',
       'tb_servo_mode_home','tb_servo_continuous_top_calibration',
       'tb_servo_pan_tilt','tb_servo_pan_tilt_pwm','tb_servo_pwm_tracking']
SOURCES=['uart_control.v','uart_rx.v','uart_tx.v','uart_packet_rx.v',
         'uart_packet_tx.v','uart_frame_command.v','uart_snapshot_mailbox.v',
         'design_top_wrapper.v','hdmi_mixer.v','osd_char_lib.v',
         'anlogic_logo_rom.v','isp_image_control.v','servo_test_control.v',
         'servo_pwm.v','servo_test_display.v']

def main():
    parser=ArgumentParser(description=__doc__)
    parser.add_argument('--iverilog',default=shutil.which('iverilog'))
    parser.add_argument('--vvp',default=shutil.which('vvp'))
    args=parser.parse_args()
    if not args.iverilog or not args.vvp:
        parser.error('Install Icarus Verilog or pass --iverilog and --vvp paths')
    root=Path(__file__).resolve().parents[1]
    hdl=root/'user_source/hdl_source'
    output=root/'sim/build'
    output.mkdir(exist_ok=True)

    def run(name):
        image=output/(name+'.vvp')
        image.unlink(missing_ok=True)
        files=[root/'sim'/(name+'.sv'),*[hdl/f for f in SOURCES]]
        command=[args.iverilog,'-g2012','-i','-Wall','-s',name,'-o',str(image),*map(str,files)]
        try:
            compile_result=subprocess.run(command,cwd=output,capture_output=True,timeout=60)
            compile_log=compile_result.stdout+compile_result.stderr
            (output/(name+'.compile.log')).write_bytes(compile_log)
            if compile_result.returncode!=0 or not image.exists():
                return {'test':name,'passed':False,'reason':'Compilation failed'}
            result=subprocess.run([args.vvp,str(image)],cwd=output,capture_output=True,timeout=120)
            log=result.stdout+result.stderr
            (output/(name+'.log')).write_bytes(log)
            passed=result.returncode==0 and b'PASS:' in log and b'FATAL:' not in log
            return {'test':name,'passed':passed,'simulator_exit':result.returncode}
        except subprocess.TimeoutExpired:
            return {'test':name,'passed':False,'reason':'Simulation timed out'}

    with ThreadPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(run,TESTS))
    (output/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    for result in results:
        print(('PASS' if result['passed'] else 'FAIL')+': '+result['test'])
    return 0 if all(result['passed'] for result in results) else 1

if __name__=='__main__':sys.exit(main())
