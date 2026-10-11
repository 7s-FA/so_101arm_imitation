"""Actual ROS messages against localhost MOCK server only, never production."""
import json
import os
from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
if os.environ.get('ROS_DOMAIN_ID')!='141':raise SystemExit('Tests require isolated mock domain 141')
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from host_pkg.action import Arm
from std_msgs.msg import String
from arm1_node import Arm1ActionClient


class RoundTrip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init();cls.node=Node('arm1_mock_test');cls.client=ActionClient(cls.node,Arm,'/arm1/data')
        cls.status=None;cls.history=[]
        def state(msg):
            cls.status=json.loads(msg.data);cls.history.append(cls.status)
        cls.sub=cls.node.create_subscription(String,'/arm1/status',state,10)
        if not cls.client.wait_for_server(timeout_sec=8):raise AssertionError('No mock server')
        cls.spin_until(lambda:cls.status is not None)
        if cls.status.get('mock') is not True:raise AssertionError('Refusing to test a hardware server')
    @classmethod
    def tearDownClass(cls):
        cls.client.destroy();cls.node.destroy_node();rclpy.try_shutdown()
    @classmethod
    def spin_until(cls,predicate,timeout=12):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and not predicate():rclpy.spin_once(cls.node,timeout_sec=.03)
        if not predicate():raise AssertionError('Timeout waiting for ROS event')
    def wait(self,future):
        self.spin_until(future.done);return future.result()
    def goal(self,command):return self.wait(self.client.send_goal_async(Arm.Goal(command=command)))
    def good(self,handle):
        self.assertTrue(handle.accepted);r=self.wait(handle.get_result_async())
        self.assertEqual(r.status,4);self.assertTrue(r.result.success);self.assertEqual(r.result.message,'IDLE')
    def test_01_unknown_rejected(self):
        self.assertFalse(self.goal('UNKNOWN').accepted)
    def test_02_a_pause_resume_duplicate_and_actual_job_result(self):
        task=self.goal('SELECT_A');self.assertTrue(task.accepted);result=task.get_result_async()
        self.assertFalse(self.goal('SELECT_B').accepted)
        self.spin_until(lambda:self.status['phase']=='WORKING')
        self.good(self.goal('EMER_STOP'))
        self.spin_until(lambda:self.status['state']=='PAUSED')
        self.assertFalse(result.done())
        self.assertFalse(self.goal('SELECT_B').accepted)
        self.good(self.goal('RESTART'))
        response=self.wait(result)
        self.assertEqual(response.status,4);self.assertTrue(response.result.success)
        self.assertTrue(any(x['phase']=='RETURNING' for x in self.history))
    def test_03_b_cancel_is_not_success_and_can_clear_pause(self):
        task=self.goal('SELECT_B');self.assertTrue(task.accepted)
        result=task.get_result_async()
        self.spin_until(lambda:self.status['command']=='SELECT_B' and self.status['phase']=='WORKING')
        cancel=self.wait(task.cancel_goal_async());self.assertTrue(cancel.goals_canceling)
        response=self.wait(result)
        self.assertEqual(response.status,5);self.assertFalse(response.result.success)
        self.assertEqual(response.result.message,'ERROR')
        self.good(self.goal('RESTART'))
    def test_04_b_completes_and_returns(self):
        self.good(self.goal('SELECT_B'))
    def test_05_team_client_reports_real_final_result(self):
        team=Arm1ActionClient()
        try:
            self.assertTrue(team._client.wait_for_server(timeout_sec=5))
            future=team.send_goal('SELECT_A');end=time.monotonic()+12
            while not future.done() and time.monotonic()<end:rclpy.spin_once(team,timeout_sec=.03)
            self.assertTrue(future.done());self.assertTrue(future.result().success)
            self.assertEqual(future.result().message,'IDLE');self.assertFalse(team.busy)
            self.assertIsNone(team.last_control_result)
        finally:team.destroy_node()


if __name__=='__main__':unittest.main(verbosity=2)
