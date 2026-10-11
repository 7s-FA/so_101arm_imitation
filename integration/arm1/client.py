"""CLI for Arm1; production goals require --hardware. Ctrl+C stops only client."""
import argparse
import os
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',nargs='?',choices=['SELECT_A','SELECT_B','EMER_STOP','RESTART'])
    p.add_argument('--check-connection',action='store_true')
    p.add_argument('--hardware',action='store_true')
    p.add_argument('--domain-id',type=int)
    p.add_argument('--timeout',type=float,default=660)
    a=p.parse_args()
    if not a.check_connection and a.command is None:p.error('Specify command or --check-connection')
    domain=a.domain_id if a.domain_id is not None else 40 if a.hardware else 141
    if domain==40 and not a.hardware:p.error('Production domain requires --hardware')
    os.environ['ROS_DOMAIN_ID']=str(domain)
    if not a.hardware:
        os.environ.pop('ROS_LOCALHOST_ONLY',None);os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    else:
        os.environ.pop('ROS_LOCALHOST_ONLY',None);os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='SUBNET'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from arm1_node import Arm1ActionClient
    from host_pkg.action import Arm
    from interface_check import validate_interface
    validate_interface(Arm)
    rclpy.init();node=Arm1ActionClient();executor=SingleThreadedExecutor();executor.add_node(node)
    try:
        if not node._client.wait_for_server(timeout_sec=5):
            print('/arm1/data: server not found');return 2
        if a.check_connection:print('/arm1/data: connected (no goal sent)');return 0
        result=node.send_goal(a.command);deadline=time.monotonic()+a.timeout
        while rclpy.ok() and not result.done() and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.1)
        if not result.done():
            print('Result timeout; robot may still be running. Use EMER_STOP to stop.');return 2
        value=result.result();print(f'{a.command}: success={value.success} message={value.message} detail={value.detail}')
        return 0 if value.success else 1
    except KeyboardInterrupt:
        print('Client stopped only; robot may still be running.');return 130
    finally:
        executor.shutdown();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':raise SystemExit(main())
