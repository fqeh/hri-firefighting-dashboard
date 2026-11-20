"""Approval gate around the existing physical Nav2 action client."""
import json
from lifecycle_msgs.srv import GetState
from hri_bringup.operator_navigation import OperatorNavigation
from hri_reasoning.lifecycle import run
from hri_reasoning.core import pose_is_fresh


class SharedNavigation(OperatorNavigation):
    def __init__(self):
        super().__init__()
        self.nav2_ready=False;self.state_request=None
        self.state_client=self.create_client(GetState,'/bt_navigator/get_state')
        self.create_timer(.2,self.watch_route)

    def stamp(self):return self.get_clock().now().nanoseconds/1e9

    def watch_route(self):
        if not self.nav2_ready and self.state_client.service_is_ready():
            if self.state_request is None:self.state_request=self.state_client.call_async(GetState.Request())
            elif self.state_request.done():
                self.nav2_ready=self.state_request.result().current_state.id==3;self.state_request=None
                if self.nav2_ready:self.report('ready',reason='Nav2 active; select a destination and approve its route')
        if (self.goal is not None or self.pending) and not self.cancel_requested and (
            not self.route or not pose_is_fresh(self.stamp(),self.route.get('source_pose_stamp'))):
            self.stop('Route telemetry is stale; stopped and requesting assistance')

    def stop(self,reason):
        self.queue=[];self.route_edges=set();self.cancel_requested=True
        if self.goal is not None:self.cancel_goal()
        self.report('approval_required',reason=reason)

    def on_route(self,msg):
        super().on_route(msg)
        if self.route and not self.route.get('navigation_allowed') and (self.goal is not None or self.pending or self.queue):
            self.stop('No sufficiently safe known route; stopped and requesting assistance')

    def control(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):raise ValueError('Command must be a JSON object')
            command=data.get('command')
            if command in ('select_destination','waypoint'):
                self.stop('Destination selected; inspect and approve the current route before motion');return
            if command=='approve_route':
                if not self.nav2_ready:
                    self.report('unavailable',reason='Nav2 has not become active');return
                if not self.route or not self.route.get('navigation_allowed'):
                    self.report('rejected',reason='No sufficiently safe known route');return
                if not pose_is_fresh(self.stamp(),self.route.get('source_pose_stamp')):
                    self.stop('Route pose is stale; request assistance');return
            super().control(msg)
        except (ValueError,TypeError) as exc:self.report('rejected',reason=str(exc))

    def dispatch(self):
        if self.queue and (not self.route or not self.route.get('navigation_allowed') or
                          not pose_is_fresh(self.stamp(),self.route.get('source_pose_stamp'))):
            self.stop('Current route is unavailable or stale');return
        super().dispatch()


def main():run(SharedNavigation)
