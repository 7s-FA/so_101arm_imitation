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
    config=load_map(a.config, require_assets=a.hardware)
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
    from host_pkg.action import Arm
    from interface_check import validate_interface
    validate_interface(Arm)
    (ROOT/'logs').mkdir(exist_ok=True)
    lock=(ROOT/('hardware.lock' if args.hardware else 'mock.lock')).open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    backend=HardwareBackend(config,args.device) if args.hardware else MockBackend(args.mock_steps)
    log=ROOT/'logs'/f'{"hardware" if args.hardware else "mock"}_{datetime.now():%Y%m%d_%H%M%S_%f}.jsonl'
    controller=Controller(backend,log_path=log,timeout=config['job_timeout_seconds'])
    controller.event('configuration',mock=not args.hardware,models=config['models'])
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
        def status(self):
            self.publisher.publish(String(data=json.dumps({**controller.status(),'mock':not args.hardware})))
        def goal(self,request):
            with self.guard:
                ticket=controller.reserve(request.command)
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
                return Arm.Result(success=ticket.success,message=word)
            finally:
                with self.guard:self.running.pop(key,None)

    node=Server();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    print(f'ARM1 READY mode={"HARDWARE" if args.hardware else "MOCK"} domain={os.environ["ROS_DOMAIN_ID"]} action={config["action_name"]}\nlog={log}',flush=True)
    if args.hardware:print('SELECT_A/B will move the real arm. EMER_STOP holds torque; server Ctrl+C releases torque.',flush=True)
    try:executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        controller.close()
        executor.shutdown(timeout_sec=5)
        node.action.destroy();node.destroy_node();rclpy.try_shutdown();lock.close()


if __name__=='__main__':main()
