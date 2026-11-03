"""Terminate only the verified project's camera-only preview, never motor owners."""
import os
from pathlib import Path
import select
import signal


def preview_process(proc, project):
    try:
        if proc.stat().st_uid != os.getuid():return False
        argv=(proc/'cmdline').read_bytes().split(b'\0')
        args=[os.fsdecode(a) for a in argv if a]
        if len(args)<2 or not Path(args[0]).name.startswith('python'):return False
        script=Path(args[1])
        if not script.is_absolute():script=(proc/'cwd').resolve()/script
        if script.resolve()!=(project/'arm3-current/arm3/camera_preview.py').resolve():return False
        devices=[os.readlink(f) for f in (proc/'fd').iterdir()]
        # Refuse to force-kill any process that also opened a motor port.
        if any(d.startswith(('/dev/tty','/dev/serial/')) for d in devices if d!='/dev/tty'):return False
        return any(d.startswith(('/dev/video','/dev/bus/usb/')) for d in devices)
    except (OSError,ValueError):return False


def release_camera_preview(project):
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
        if not preview_process(proc,project):continue
        try:fd=os.pidfd_open(int(proc.name))
        except ProcessLookupError:continue
        try:
            # Revalidate after opening pidfd; never signal a recycled PID.
            if not preview_process(proc,project):continue
            poll=select.poll();poll.register(fd,select.POLLIN)
            signal.pidfd_send_signal(fd,signal.SIGTERM)
            if not poll.poll(2000):
                signal.pidfd_send_signal(fd,signal.SIGKILL)
                if not poll.poll(2000):raise RuntimeError('Camera preview termination unconfirmed')
            print(f'Camera-only preview released: pid={proc.name}',flush=True)
        except ProcessLookupError:pass
        finally:os.close(fd)
