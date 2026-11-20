"""Operator waypoints forwarded to the actual Nav2 action server."""
import json
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from .controls import waypoint_coordinates


class OperatorNavigation(Node):
    def __init__(self):
        super().__init__('operator_navigation')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.pub = self.create_publisher(String, '/hri/navigation', 10)
        self.create_subscription(String, '/hri/control', self.control, 10)
        self.goal = None
        self.pending = False
        self.cancel_requested = False
        self.current_waypoint=None
        self.queue=[];self.route=None;self.route_edges=set();self.risks={}
        self.create_subscription(String,'/hri/robot_route',self.on_route,10)

    def on_route(self,msg):
        try:
            route=json.loads(msg.data)
            risks={e:s for e,s in route.get('edge_states',{}).items() if s in ('blocked','hazardous')}
            changed=any(e in self.route_edges and self.risks.get(e)!=s for e,s in risks.items())
            self.route=route;self.risks=risks
            if changed and (self.goal is not None or self.pending):
                self.queue=[];self.cancel_requested=True
                self.report('reapproval_required',reason='A newly observed hazard affects the approved route',revision=route['revision'])
                if self.goal is not None:self.cancel_goal()
        except (ValueError,KeyError,TypeError) as exc:self.report('rejected',reason=str(exc))

    def dispatch(self):
        if self.pending or self.goal is not None or not self.queue:return
        if not self.client.server_is_ready():
            self.report('unavailable',reason='Nav2 action server is not ready');self.queue=[];return
        x,y,frame=self.queue.pop(0)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id=frame
        goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=float(x);goal.pose.pose.position.y=float(y);goal.pose.pose.orientation.w=1.
        self.pending=True;self.cancel_requested=False
        self.current_waypoint={'x':x,'y':y,'frame':frame}
        self.report('requested',x=x,y=y,frame=frame,remaining_waypoints=len(self.queue))
        self.client.send_goal_async(goal, feedback_callback=self.feedback).add_done_callback(self.accepted)

    def feedback(self, message):
        feedback=message.feedback
        position=feedback.current_pose.pose.position
        distance=feedback.distance_remaining
        if distance==0. and self.current_waypoint and math.hypot(position.x-self.current_waypoint['x'],position.y-self.current_waypoint['y'])>.4:distance=None
        self.report('navigating', distance_remaining=distance, recoveries=feedback.number_of_recoveries,
            navigation_time=feedback.navigation_time.sec+feedback.navigation_time.nanosec/1e9,
            remaining_waypoints=len(self.queue), current_waypoint=self.current_waypoint)

    def report(self, state, **data):
        self.pub.publish(String(data=json.dumps({'stamp':self.get_clock().now().nanoseconds/1e9,'state': state, 'current_waypoint':self.current_waypoint, **data})))

    def control(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and data.get('command') == 'cancel_waypoint':
                self.queue=[];self.route_edges=set()
                self.cancel_requested = True
                if self.goal is not None:
                    self.cancel_goal()
                elif self.pending:
                    self.report('canceling', reason='waiting for goal acceptance before cancellation')
                else:
                    self.cancel_requested = False
                    self.report('idle', reason='no active waypoint')
                return
            if isinstance(data,dict) and data.get('command')=='approve_route':
                if not self.route or data.get('revision')!=self.route.get('revision'):
                    raise ValueError('Route changed or unavailable; approve its current revision')
                if not self.route.get('waypoints') or self.route.get('safety')=='unavailable':
                    raise ValueError('No connected route is available for approval')
                self.queue=[(p['x'],p['y'],'world') for p in self.route['waypoints']]
                self.route_edges=set(self.route.get('edges',[]))
                self.report('route_approved',revision=self.route['revision'],safety=self.route['safety'])
                if self.pending or self.goal is not None:
                    self.cancel_requested=True
                    if self.goal is not None:self.cancel_goal()
                else:self.dispatch()
                return
            coordinates = waypoint_coordinates(data)
            if coordinates is None:
                return
            x, y = coordinates
            self.queue=[(x,y,'odom')];self.route_edges=set()
            if self.pending or self.goal is not None:
                self.cancel_requested=True
                if self.goal is not None:self.cancel_goal()
            else:self.dispatch()
        except (ValueError, KeyError, TypeError) as error:
            self.report('rejected', reason=str(error))
        except Exception as error:
            self.report('failed', reason=f'Nav2 request failed: {error}')

    def accepted(self, future):
        self.pending = False
        try:
            handle = future.result()
        except Exception as error:
            self.report('failed', reason=f'Nav2 goal response failed: {error}')
            return
        if not handle.accepted:
            self.report('goal_rejected', reason='Nav2 rejected the goal')
            return
        self.goal = handle
        self.report('navigating')
        try:
            handle.get_result_async().add_done_callback(self.finished)
            if self.cancel_requested and self.goal is not None:
                self.cancel_goal()
        except Exception as error:
            self.goal = None
            self.report('failed', reason=f'Nav2 result request failed: {error}')

    def finished(self, future):
        self.goal = None
        try:
            result = future.result()
        except Exception as error:
            self.report('failed', reason=f'Nav2 result failed: {error}')
            return
        state = {4: 'succeeded', 5: 'canceled'}.get(result.status, 'failed')
        self.report(state, action_status=result.status)
        if state in ('succeeded','canceled'):self.dispatch()
        else:self.queue=[]

    def cancel_goal(self):
        self.report('canceling')
        self.goal.cancel_goal_async().add_done_callback(self.cancel_response)

    def cancel_response(self, future):
        try:
            response = future.result()
            if not response.goals_canceling:
                self.report('cancel_rejected', reason='Nav2 did not accept cancellation')
        except Exception as error:
            self.report('cancel_failed', reason=str(error))


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node = OperatorNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
