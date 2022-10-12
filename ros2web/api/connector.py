from collections import defaultdict
from typing import Dict, Set
from typing import Optional, Callable

import asyncio
import itertools
import threading

import rclpy.node
import launch.logging
from ros2node.api import get_publisher_info, get_subscriber_info, get_service_server_info

from ros2web_interfaces.msg import WSMsg, WSMsgData, WSMsgType
from ros2web_interfaces.srv import HTTP

TIMER_PERIOD = 10  # sec.


def pub_topic_exists(*, topic_name: str, remote_node_name: str, ros_node: rclpy.node.Node) -> bool:
    topic_names_and_types = get_publisher_info(
        node=ros_node, remote_node_name=remote_node_name)
    topic_names = [topic_name
                   for (topic_name, topic_types) in topic_names_and_types
                   if 'ros2web_interfaces/msg/WSMsg' in topic_types]

    return topic_name in topic_names


def sub_topic_exists(*, topic_name: str, remote_node_name: str, ros_node: rclpy.node.Node) -> bool:
    topic_names_and_types = get_subscriber_info(
        node=ros_node, remote_node_name=remote_node_name)
    topic_names = [topic_name
                   for (topic_name, topic_types) in topic_names_and_types
                   if 'ros2web_interfaces/msg/WSMsg' in topic_types]

    return topic_name in topic_names


def service_exists(*, srv_name: str, remote_node_name: str, ros_node: rclpy.node.Node) -> bool:
    service_names_and_types = get_service_server_info(
        node=ros_node, remote_node_name=remote_node_name)

    srv_names = [srv_name
                 for (srv_name, srv_types) in service_names_and_types
                 if 'ros2web_interfaces/srv/HTTP' in srv_types]

    return srv_name in srv_names


class NodeConnector:

    def __init__(self, remote_node_name,
                 *, ros_node: rclpy.node.Node, ros2web_name: str = 'ros2web') -> None:

        self.__ros2web_name = ros2web_name
        self.__method_path = f'/{self.__ros2web_name}/ws'

        self.__remote_node_name = remote_node_name
        self.__ros_node = ros_node

        self.__logger = launch.logging.get_logger('NodeConnector')
        self.__lock = threading.Lock()
        self.__subscription: rclpy.node.Subscription = None
        self.__publisher: rclpy.node.Publisher = None
        self.__timer: rclpy.node.Timer = None

        self.__recv = None
        self.__is_running = False

        self.__ws_ids: Dict[str, Set[str]] = {}

    def connect(self):
        pub_topic = f'/{self.__ros2web_name}/{self.__remote_node_name}/pub'
        sub_topic = f'/{self.__ros2web_name}/{self.__remote_node_name}/sub'

        try:
            if pub_topic_exists(topic_name=pub_topic, remote_node_name=self.__remote_node_name, ros_node=self.__ros_node) and \
                    sub_topic_exists(topic_name=sub_topic, remote_node_name=self.__remote_node_name, ros_node=self.__ros_node):

                if not pub_topic_exists(topic_name=sub_topic, remote_node_name=self.__ros2web_name, ros_node=self.__ros_node):
                    self.__publisher = self.__ros_node.create_publisher(
                        WSMsg, sub_topic, 10)
                elif self.__publisher is None:
                    for publisher in self.__ros_node.publishers:
                        if sub_topic == publisher.topic_name:
                            self.__publisher = publisher
                            break

                    if self.__publisher is None:
                        raise RuntimeError('Failed to get publisher.')

                if not sub_topic_exists(topic_name=pub_topic, remote_node_name=self.__ros2web_name, ros_node=self.__ros_node):
                    self.__subscription = self.__ros_node.create_subscription(WSMsg, pub_topic,
                                                                              self.__on_subscribe, 10)
                elif self.__subscription is None:
                    for subscription in self.__ros_node.subscriptions:
                        if pub_topic == subscription.topic_name:
                            self.__subscription = subscription
                            break

                    if self.__subscription is None:
                        raise RuntimeError('Failed to get subscription.')

                self.__timer = self.__ros_node.create_timer(
                    TIMER_PERIOD, self.__interval_check_srv_name)
                self.__is_running = True
        except rclpy.node.NodeNameNonExistentError as e:
            self.__logger.error(e)
            self.__is_running = False

    def __interval_check_srv_name(self):
        try:
            routes = set()
            for route, value in self.__ws_ids.items():
                if len(value) > 0:
                    srv_name = f'{self.__method_path}{route}'
                    if service_exists(srv_name=srv_name, remote_node_name=self.__remote_node_name, ros_node=self.__ros_node):
                        continue
                routes.add(route)

            with self.__lock:
                for route in routes:
                    self.__logger.info('inactive: {}'.format(route))
                    del self.__ws_ids[route]

            ws_ids = set(itertools.chain.from_iterable(
                [self.__ws_ids[route] for route in routes]))
            self.__dis_conn(ws_ids)

        except rclpy.node.NodeNameNonExistentError as e:
            self.__logger.debug(e)
            self.__disconnect()

    def __disconnect(self):
        self.__logger.info('inactive node: {}'.format(self.__remote_node_name))
        self.__is_running = False

        if self.__timer is not None:
            self.__ros_node.destroy_timer(self.__timer)

        if self.__publisher is not None:
            self.__ros_node.destroy_publisher(self.__publisher)
        if self.__subscription is not None:
            self.__ros_node.destroy_subscription(self.__subscription)

        ws_ids = set(itertools.chain.from_iterable(
            [ws_ids for ws_ids in self.__ws_ids.values()]))
        self.__dis_conn(ws_ids)

        self.__publisher = None
        self.__subscription = None
        self.__timer = None

    def register(self, route: str, ws_id: str):
        with self.__lock:
            if route in self.__ws_ids:
                self.__ws_ids[route].add(ws_id)
            else:
                self.__ws_ids[route] = set()
                self.__ws_ids[route].add(ws_id)

    def unregister(self, route: str, ws_id: str):
        with self.__lock:
            if route in self.__ws_ids:
                self.__ws_ids[route].discard(ws_id)
                if len(self.__ws_ids[route]) == 0:
                    del self.__ws_ids[route]

    def publish(self, msg: WSMsg):
        if self.__publisher is not None:
            self.__publisher.publish(msg)

    def __on_subscribe(self, msg: WSMsg):
        if self.__recv is not None:
            if msg.ws_id == '' and msg.route == '':
                self.__logger.warning(
                    "WSMsg requires a property value of ws_id or route.".format(msg))
                return
            ws_ids = set()
            if msg.ws_id == '':
                ws_ids = self.__ws_ids.get(msg.route, [])
            else:
                ws_ids.add(msg.ws_id)

            self.__recv(msg, ws_ids)

    def shutdown(self):
        self.__logger.info('shutdown: {}'.format(self.__remote_node_name))
        self.__is_running = False

        if self.__timer is not None:
            self.__ros_node.destroy_timer(self.__timer)

        if self.__publisher is not None:
            self.__ros_node.destroy_publisher(self.__publisher)

        if self.__subscription is not None:
            self.__ros_node.destroy_subscription(self.__subscription)

        self.__ws_ids = defaultdict(set)
        self.__publisher = None
        self.__subscription = None
        self.__timer = None

    def is_running(self) -> bool:
        return self.__is_running

    @property
    def recv(self):
        return self.__recv

    @recv.setter
    def recv(self, func: Callable[[WSMsg], None]):
        if callable(func):
            self.__recv = func

    @property
    def dis_conn(self):
        return self.__dis_conn

    @dis_conn.setter
    def dis_conn(self, func: Callable[[WSMsg], None]):
        if callable(func):
            self.__dis_conn = func
