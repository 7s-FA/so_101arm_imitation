"""Arm1 ROS 2 server. Mock by default; --hardware explicitly enables real goals."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import threading
import time

from backend import load_map, HardwareBackend
from engine import Controller, MockBackend, JOBS
from gc_watch import GcWatch

ROOT=Path(__file__).resolve().parent


def arguments():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hardware',action='store_true',help='Accept real robot motion goals; no motion until a goal')
    p.add_argument('--config',type=Path,default=ROOT/'model-map.json')
    p.add_argument('--domain-id',type=int)
    p.add_argument('--device',choices=['cpu','cuda'],default='cuda')
    p.add_argument('--mock-steps',type=int,default=30)
    a=p.parse_args()
    if a.mock_steps<1:p.error('--mock-steps must be positive')
    config=load_map(a.config)
    domain=a.domain_id if a.domain_id is not None else config['hardware_domain_id' if a.hardware else 'mock_domain_id']
    if not 0<=domain<=232:p.error('Invalid ROS domain')
    if not a.hardware and domain==config['hardware_domain_id']:
        p.error('Mock cannot run in the production domain')
    os.environ['ROS_DOMAIN_ID']=str(domain)
    if not a.hardware:
        os.environ.pop('ROS_LOCALHOST_ONLY',None)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    else:
        os.environ.pop('ROS_LOCALHOST_ONLY',None)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='SUBNET'
    return a,config


def main():
    args,config=arguments()
    # Imports after selecting domain; mock never imports robot/camera SDKs.
    import rclpy
    from rclpy.action import ActionServer,GoalResponse,CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor,ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import CompressedImage
    from host_pkg.action import Arm
    from interface_check import validate_interface
    validate_interface(Arm)
    (ROOT/'logs').mkdir(exist_ok=True)
    lock=(ROOT/('hardware.lock' if args.hardware else 'mock.lock')).open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    backend=HardwareBackend(config,args.device) if args.hardware else MockBackend(args.mock_steps)
    # Cameras only (no motors) so the vision judge and GUI see the scene before the first goal.
    if args.hardware:backend.start_cameras()
    log=ROOT/'logs'/f'{"hardware" if args.hardware else "mock"}_{datetime.now():%Y%m%d_%H%M%S_%f}.jsonl'
    controller=Controller(backend,log_path=log,timeout=config['job_timeout_seconds'],
                          completion_delay=config.get('completion_delay_seconds',5.0))
    controller.event('configuration',mock=not args.hardware,models=config['models'],
                     completion_delay_seconds=controller.completion_delay)
    rclpy.init()

    class Server(Node):
        def __init__(self):
            super().__init__('arm1_controller')
            self.pending={};self.running={};self.guard=threading.RLock()
            self.action=ActionServer(self,Arm,config['action_name'],self.execute,
                goal_callback=self.goal,cancel_callback=self.cancel,
                callback_group=ReentrantCallbackGroup())
            self.publisher=self.create_publisher(String,'/arm1/status',10)
            self.create_timer(.5,self.status)
            # GUI feed (read-only): judge result, command history and an annotated overview image.
            self.history=[];self.vision_publisher=self.create_publisher(String,'/arm1/vision',10)
            self.image_publisher=self.create_publisher(CompressedImage,'/arm1/vision/image',2)
            self.create_timer(.5,self.vision)
            self.create_timer(1.,self.gc_report)
        def gc_report(self):
            for pause in gc_watch.drain():controller.event('gc',**pause)
        def note(self,**event):
            with self.guard:
                self.history=(self.history+[{'time':time.time(),**event}])[-20:]
        def status(self):
            self.publisher.publish(String(data=json.dumps({**controller.status(),'mock':not args.hardware})))
        def vision(self):
            import cv2
            report={'time':time.time(),'mock':not args.hardware,'status':controller.status(),'camera':None,
                    'job':getattr(backend,'vision_job',None)}
            with self.guard:report['history']=list(self.history)
            # Measured joints for the GUI's virtual robot (last motor sample; only while the motor bus is open).
            if getattr(backend,'bus',None) is not None and getattr(backend,'current',None):report['joints_raw']=dict(backend.current)
            judge,cameras=getattr(backend,'judge',None),getattr(backend,'cameras',None)
            frame=None
            if judge is not None and cameras is not None:
                try:frame=cameras.snapshot()['overview'][0]
                except Exception as exc:report['camera']=f'overview unavailable: {exc}'
            if frame is not None:
                result=judge.evaluate(frame)
                preview={c:judge.select(c,result) for c in ('SELECT_A','SELECT_B')}
                report.update(camera='ok',sets=result['sets'],targets=result['targets'],cells=result['cells'],preview=preview)
                # Live target-pallet measurement (jig_cells, ~4ms) for the GUI: the same numbers PRECHECK uses for a job.
                jc=getattr(backend,'jig_cells',None)
                if jc is not None:
                    try:report['burger_live']=jc.measure(cv2.cvtColor(frame,cv2.COLOR_RGB2BGR))
                    except Exception as exc:report['burger_live']={'ok':False,'reason':repr(exc)}
                job=report['job'];shown=job['selection'] if job and report['status']['command'] else None
                ok,jpeg=cv2.imencode('.jpg',judge.annotate(frame,result,shown),[cv2.IMWRITE_JPEG_QUALITY,80])
                if ok:self.image_publisher.publish(CompressedImage(format='jpeg',data=jpeg.tobytes()))
            self.vision_publisher.publish(String(data=json.dumps(report,ensure_ascii=False,default=str)))
        def goal(self,request):
            with self.guard:
                ticket=controller.reserve(request.command)
                self.note(event='received',command=request.command,accepted=ticket is not None)
                if ticket is None:return GoalResponse.REJECT
                self.pending[request.command]=ticket
                return GoalResponse.ACCEPT
        def cancel(self,handle):
            with self.guard:
                key=bytes(handle.goal_id.uuid)
                ticket=self.running.get(key) or self.pending.get(handle.request.command)
                if ticket and ticket.command in JOBS and controller.cancel(ticket):return CancelResponse.ACCEPT
                return CancelResponse.REJECT
        def execute(self,handle):
            key=bytes(handle.goal_id.uuid)
            with self.guard:
                ticket=self.pending.pop(handle.request.command)
                self.running[key]=ticket
            controller.activate(ticket)
            previous=None
            try:
                while not ticket.done.wait(.05):
                    state=controller.status()['state']
                    # STOPPING is still RUNNING until pose feedback confirms stop.
                    word=state if state in ('IDLE','PAUSED','ERROR') else 'RUNNING'
                    if word!=previous:
                        handle.publish_feedback(Arm.Feedback(message=word));previous=word
                word='IDLE' if ticket.success else 'ERROR'
                if ticket.command=='EMER_STOP' and ticket.success:
                    handle.publish_feedback(Arm.Feedback(message='PAUSED'))
                elif ticket.command=='RESTART' and ticket.success:
                    state=controller.status()['state']
                    handle.publish_feedback(Arm.Feedback(message='IDLE' if state=='IDLE' else 'RUNNING'))
                else:handle.publish_feedback(Arm.Feedback(message=word))
                if ticket.cancelled and handle.is_cancel_requested:handle.canceled()
                elif ticket.success:handle.succeed()
                else:handle.abort()
                self.get_logger().info(f'{ticket.command}: {word}: {ticket.detail}')
                self.note(event='result',command=ticket.command,success=ticket.success,message=word,detail=ticket.detail)
                return Arm.Result(success=ticket.success,message=word)
            finally:
                with self.guard:self.running.pop(key,None)

    gc_watch=GcWatch()
    node=Server();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    # Everything is loaded and the motor bus is not open yet: a ~0.5 s full collection here is harmless.
    controller.event('gc_freeze',**gc_watch.freeze());gc_watch.install()
    print(f'ARM1 READY mode={"HARDWARE" if args.hardware else "MOCK"} domain={os.environ["ROS_DOMAIN_ID"]} action={config["action_name"]}\nlog={log}',flush=True)
    if args.hardware:print('SELECT_A/B will move the real arm. EMER_STOP holds torque; server Ctrl+C releases torque.',flush=True)
    try:executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        gc_watch.remove()
        controller.close()
        executor.shutdown(timeout_sec=5)
        node.action.destroy();node.destroy_node();rclpy.try_shutdown();lock.close()


if __name__=='__main__':main()
