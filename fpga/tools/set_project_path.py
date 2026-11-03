"""Set the TD project-directory metadata without changing design inputs."""
from pathlib import Path
import re

def configure(project):
    before=project.read_bytes()
    directory=project.parent.resolve().as_posix().encode('utf-8')
    after,count=re.subn(rb'(<Project\b[^>]*\bPath=")[^"]*(")',
                       lambda m:m.group(1)+directory+m.group(2),before,count=1)
    if count!=1:
        raise RuntimeError('Expected exactly one Project Path attribute')
    if after!=before:project.write_bytes(after)
    return project.parent.resolve()

if __name__=='__main__':
    path=Path(__file__).resolve().parents[1]/'td_project/camera_to_dsi_display.al'
    print('Project directory:',configure(path))
