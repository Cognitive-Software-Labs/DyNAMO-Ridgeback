#!/usr/bin/env python3
"""RViz markers for leader following: tracked people, the leader and the lock state.

Publishes ``leader_follow/markers`` (MarkerArray):

* every tracked person as a grey cylinder with a velocity arrow and track id;
* the leader as a green cylinder while LOCKED, amber while UNCERTAIN, red when LOST;
* lock state and follow mode as text above the robot's leader.

Visualisation only; nothing in the follow stack reads it.
"""
import math

STATE_NAMES = {0: 'SEARCHING', 1: 'LOCKED', 2: 'UNCERTAIN', 3: 'LOST'}
STATE_COLOURS = {0: (.6, .6, .6), 1: (.1, .85, .2), 2: (1., .7, 0.), 3: (.9, .1, .1)}


def main():
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    from leader_follow_interfaces.msg import LeaderState, People

    class FollowMarkers(Node):
        def __init__(self):
            super().__init__('follow_markers')
            self.publisher = self.create_publisher(MarkerArray, 'leader_follow/markers', 10)
            self.leader = None
            self.status = ''
            self.create_subscription(LeaderState, 'leader_follow/leader', self.on_leader, 10)
            self.create_subscription(String, 'leader_follow/follow_status', self.on_status, 10)
            self.create_subscription(People, 'leader_follow/people', self.on_people, 10)

        def on_leader(self, msg):
            self.leader = msg

        def on_status(self, msg):
            self.status = msg.data

        def marker(self, header, ns, index, kind, colour, alpha=.8):
            m = Marker()
            m.header = header
            m.ns, m.id, m.type, m.action = ns, index, kind, Marker.ADD
            m.color.r, m.color.g, m.color.b = colour
            m.color.a = alpha
            m.pose.orientation.w = 1.
            m.lifetime.nanosec = 400_000_000
            return m

        def on_people(self, msg):
            leader = self.leader
            leader_id = leader.track_id if leader is not None and leader.has_leader else None
            state = leader.state if leader is not None else 0
            out = MarkerArray()
            for person in msg.people:
                is_leader = person.id == leader_id
                colour = STATE_COLOURS.get(state, (.6, .6, .6)) if is_leader else (.55, .55, .6)
                body = self.marker(msg.header, 'people', person.id, Marker.CYLINDER, colour, .9 if is_leader else .6)
                body.pose.position.x, body.pose.position.y, body.pose.position.z = person.position.x, person.position.y, .85
                body.scale.x = body.scale.y = .5 if is_leader else .4
                body.scale.z = 1.7
                out.markers.append(body)
                speed = math.hypot(person.velocity.x, person.velocity.y)
                if speed > .1:
                    arrow = self.marker(msg.header, 'velocity', person.id, Marker.ARROW, colour)
                    start = Point(x=person.position.x, y=person.position.y, z=1.8)
                    end = Point(x=person.position.x + person.velocity.x, y=person.position.y + person.velocity.y, z=1.8)
                    arrow.points = [start, end]
                    arrow.scale.x, arrow.scale.y, arrow.scale.z = .06, .14, .14
                    out.markers.append(arrow)
                label = self.marker(msg.header, 'ids', person.id, Marker.TEXT_VIEW_FACING, (1., 1., 1.), 1.)
                label.pose.position.x, label.pose.position.y, label.pose.position.z = person.position.x, person.position.y, 2.0
                label.scale.z = .25
                label.text = f'{"LEADER " if is_leader else ""}{person.id}'
                out.markers.append(label)
            if leader is not None and leader.has_leader:
                text = self.marker(leader.header, 'state', 0, Marker.TEXT_VIEW_FACING,
                                   STATE_COLOURS.get(leader.state, (1., 1., 1.)), 1.)
                text.pose.position.x, text.pose.position.y, text.pose.position.z = leader.position.x, leader.position.y, 2.4
                text.scale.z = .3
                text.text = f'{STATE_NAMES.get(leader.state, "?")} | {self.status.split(":")[0]}'
                out.markers.append(text)
            self.publisher.publish(out)

    rclpy.init()
    node = FollowMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
