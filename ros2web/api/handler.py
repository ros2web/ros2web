from typing import Dict, List, Set, Any, Tuple
from typing import Optional, Callable, NoReturn

import asyncio
import aiohttp
from aiohttp import web
from aiohttp import WSCloseCode
import itertools
import weakref

import rclpy.node
import launch.logging
from ros2service.api import get_service_names_and_types

from ros2web_interfaces.srv import HTTP, ServerInfo
from ros2web_interfaces.msg import StatusMsg, StatusType
from ros2web_interfaces.msg import HTTPStatusCode, ContentType
from ros2web_interfaces.msg import WSMsg, WSMsgData, WSMsgType

from .connector import NodeConnector
from .ros_executor import ROSExecutor


class HandlerNode:
    def __init__(self, *, node_name: str, directory: str,
                 ip_address: str, port: int, token: str,
                 loop: asyncio.AbstractEventLoop) -> None:

        self.__node_name = node_name
        self.__directory = directory
        self.__ip_address = ip_address
        self.__port = port
        self.__token = token
        self.__loop = loop

        self.__ws_lock = asyncio.Lock()
        self.__websocket_dict: Dict[str, web.WebSocketResponse] \
            = weakref.WeakValueDictionary()
            
        self.__connectors: Dict[str, NodeConnector] = {}
        self.__ros: ROSExecutor = None
        self.__info_srv = None
        self.__broadcast_publisher = None
        self.__srv_clients = {}

        self.__logger = launch.logging.get_logger('WebHandler')

    async def startup(self):

        self.__ros = ROSExecutor(node_name=self.__node_name, loop=self.__loop)

        INFO_SRV = f'{self.__node_name}/info'
        BROADCAST_MSG = f'{self.__node_name}/broadcast'

        self.__info_srv = self.__ros.node.create_service(ServerInfo,
                                                         INFO_SRV, self.__info_srv_handler)

        self.__broadcast_publisher = self.__ros.node.create_publisher(
            StatusMsg, BROADCAST_MSG, 10)

        msg = StatusMsg()
        msg.node_name = self.__node_name
        msg.type = StatusType.STARTUP
        self.__broadcast_publisher.publish(msg)

    async def shutdown(self):
        msg = StatusMsg()
        msg.node_name = self.__node_name
        msg.type = StatusType.SHUTDOWN
        self.__broadcast_publisher.publish(msg)

        for conn in self.__connectors.values():
            conn.shutdown()

        async with self.__ws_lock:
            websockets = [ws for ws in self.__websocket_dict.values()]
        
        if len(websockets) > 0:
            await asyncio.wait([client.close(code=WSCloseCode.GOING_AWAY, message='Server shutdown')
                                for client in websockets if client.closed is False])
        
    async def cleanup(self):
        self.__ros.node.destroy_service(self.__info_srv)
        self.__ros.node.destroy_publisher(self.__broadcast_publisher)
        self.__ros.shutdown()
        await asyncio.sleep(0.2)

    def __info_srv_handler(self, request: ServerInfo.Request,
                           response: ServerInfo.Response) -> NoReturn:
        response.directory = self.__directory
        response.ip_address = self.__ip_address
        response.port = self.__port
        response.token = self.__token

        return response

    def __get_srv_name(self, method: str, path: str) -> Tuple[Optional[str],
                                                              Optional[str]]:
        method = method.lower()
        method_path = f'/{self.__node_name}/{method}'
        http_route = None
        srv_name = None

        service_names_and_types = get_service_names_and_types(
            node=self.__ros.node)
        http_srv_names = [service_name.replace(method_path, '')
                          for (service_name, service_type) in service_names_and_types
                          if 'ros2web_interfaces/srv/HTTP' in service_type
                          and service_name.startswith(method_path)]

        routes = sorted(http_srv_names, reverse=True)
        for route in routes:
            if path.startswith(route):
                http_route = route
                srv_name = f'{method_path}{route}'
                break

        return srv_name, http_route

    def __get_service_client(self, srv_name: str, timeout_sec: int = 1.0) -> Optional[rclpy.node.Client]:

        srv_client = self.__srv_clients.get(srv_name)
        if srv_client is None:
            srv_client = self.__ros.node.create_client(HTTP, srv_name)
            self.__srv_clients[srv_name] = srv_client

        ready = srv_client.wait_for_service(timeout_sec=timeout_sec)
        if not ready:
            del self.__srv_clients[srv_name]
            self.__ros.node.destroy_client(srv_client)
            self.__logger.error(
                'Wait for service timed out. ({})'.format(srv_name))
            return None

        return srv_client

    async def __convert_http_request(self, request: web.Request) -> HTTP.Request:

        http_request = HTTP.Request()

        http_request.method = request.method
        http_request.path = request.path
        http_request.query = request.query_string
        http_request.content_type = request.content_type
        
        if request.body_exists:
            text = await request.text() # Returns str with body content.
            http_request.text = text
            # byte_body = await request.read() # returns bytes object with body content.
            # http_request.body = byte_body
        
        headers = list(itertools.chain.from_iterable(
            list(request.headers.items())))
        http_request.headers = headers
        # headers = dict(zip(headers[0::2], headers[1::2]))

        return http_request

    def __convert_file_response(self, response: HTTP.Response) -> web.Response:

        # chunk_size: int = 256 * 1024
        status = response.status if response.status > 0 else 200
        reason = response.reason if response.reason != "" else None

        return web.FileResponse(response.file_path, status=status, reason=reason)

    def __convert_web_response(self, response: HTTP.Response) -> web.Response:
        body = None if response.body == b'\x00' else response.body
        status = response.status if response.status > 0 else 200
        reason = response.reason if response.reason != "" else None
        text = response.text if response.text != "" else None
        # headers: LooseHeaders | None = None
        content_type = response.content_type if response.content_type != "" else None
        charset = response.charset if response.charset != "" else None

        return web.Response(body=body, text=text, status=status, reason=reason,
                            content_type=content_type, charset=charset)

    async def __ws_request_open(self, srv_client: rclpy.node.Client,
                                http_request: HTTP.Request, ws_id: str) -> HTTP.Response:

        http_request.ws = ['open', ws_id]
        http_response: HTTP.Response = await srv_client.call_async(http_request)

        if http_response is None:
            http_response = HTTP.Response()
            http_response.status = HTTPStatusCode.HTTP_INTERNAL_SERVER_ERROR

        if http_response.status == 0:
            http_response.status = HTTPStatusCode.HTTP_OK

        return http_response

    async def __ws_request_close(self, srv_client: rclpy.node.Client,
                                 http_request: HTTP.Request, ws_id: str) -> HTTP.Response:
        http_request.ws = ['close', ws_id]
        http_response: HTTP.Response = await srv_client.call_async(http_request)

        if http_response is None:
            http_response = HTTP.Response()
            http_response.status = HTTPStatusCode.HTTP_INTERNAL_SERVER_ERROR

        if http_response.status == 0:
            http_response.status = HTTPStatusCode.HTTP_OK

        return http_response

    async def __ws_send_str(self, data: str, ws_ids: Set):
        async with self.__ws_lock:
            websockets = [self.__websocket_dict.get(ws_id) for ws_id in ws_ids
                          if self.__websocket_dict.get(ws_id) is not None]

        if len(websockets) > 0:
            await asyncio.wait([ws.send_str(data)
                                for ws in websockets if ws.closed is False])

    def __ws_receve(self, msg: WSMsg, ws_ids: Set):
        
        if msg.type == WSMsgType.TEXT:
            data = msg.data.str
            asyncio.run_coroutine_threadsafe(
                self.__ws_send_str(data, ws_ids), self.__loop)
        elif msg.type == WSMsgType.BINARY:
            pass
        elif msg.type == WSMsgType.CLOSE:
            pass

    async def __ws_close(self, ws_ids: Set):
        async with self.__ws_lock:
            websockets = [self.__websocket_dict.get(ws_id) for ws_id in ws_ids
                          if self.__websocket_dict.get(ws_id) is not None]
        
        if len(websockets) > 0:
            await asyncio.wait([client.close(code=WSCloseCode.GOING_AWAY, message='Server shutdown')
                                for client in websockets if client.closed is False])

    def __ws_disconn(self, ws_ids: Set):
        asyncio.run_coroutine_threadsafe(self.__ws_close(ws_ids), self.__loop)

    def __get_connector(self, remote_node_name: str) -> NodeConnector:
        connector = self.__connectors.get(remote_node_name)
        if connector is not None:
            if not connector.is_running():
                connector.connect()
        else:
            connector = NodeConnector(remote_node_name, ros_node=self.__ros.node,
                                      ros2web_name=self.__node_name)
            self.__connectors[remote_node_name] = connector

            connector.recv = self.__ws_receve
            connector.dis_conn = self.__ws_disconn
            connector.connect()
        return connector

    async def ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws_id = request.headers.get('Sec-WebSocket-Key')

        srv_name, route = self.__get_srv_name('ws', request.path)
        if route is None:
            self.__logger.error(
                'There is no route. ({})'.format(request.path))
            raise web.HTTPNotFound()

        http_request = await self.__convert_http_request(request)
        http_request.srv_name = srv_name
        http_request.route = route

        srv_client = self.__get_service_client(srv_name)
        if srv_client is None:
            raise web.HTTPInternalServerError()
        http_response = await self.__ws_request_open(srv_client, http_request, ws_id)
        if http_response.status == HTTPStatusCode.HTTP_INTERNAL_SERVER_ERROR:
            raise web.HTTPInternalServerError()
        if http_response.status != HTTPStatusCode.HTTP_OK:
            raise web.HTTPInternalServerError()
        if http_response.node_name == '':
            raise web.HTTPInternalServerError()

        connector = self.__get_connector(http_response.node_name)
        if not connector.is_running():
            raise web.HTTPInternalServerError()

        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        connector.register(http_request.route, ws_id)
        
        async with self.__ws_lock:
            self.__websocket_dict[ws_id] = ws
        try:
            async for ws_msg in ws:
                msg = WSMsg()
                if ws_msg.type == aiohttp.WSMsgType.TEXT:
                    msg.route = route
                    msg.ws_id = ws_id
                    msg.type = WSMsgType.TEXT
                    msg.data.str = ws_msg.data
                    msg.extra = ws_msg.extra
                    connector.publish(msg)
                # elif ws_msg.type == aiohttp.WSMsgType.BINARY:
                #     msg.type = MsgType.BINARY
                #     msg.data.bytes = ws_msg.data
                
        except RuntimeError as e:
            self.__logger.error(f"RuntimeError: {e}")
        except ValueError as e:
            self.__logger.error(f"ValueError: {e}")
        except TypeError as e:
            self.__logger.error(f"TypeError: {e}")
        finally:
            async with self.__ws_lock:
                del self.__websocket_dict[ws_id]

            connector.unregister(http_request.route, ws_id)

            srv_client = self.__get_service_client(srv_name)
            if srv_client is None:
                raise web.HTTPInternalServerError()
            http_response = await self.__ws_request_close(srv_client, http_request, ws_id)
            if http_response.status != HTTPStatusCode.HTTP_OK:
                raise web.HTTPInternalServerError()
        
        return ws

    async def web_handler(self, request: web.Request) -> web.StreamResponse:

        ws_key = request.headers.get('Sec-WebSocket-Key')
        if ws_key is not None:
            return await self.ws_handler(request)

        # time_start = time.perf_counter()
        # elapsed_time = time.perf_counter() - time_start
        # print('elapsed time: ', elapsed_time)
        self.__logger.info('request: {}'.format(request.path))
        
        srv_name, route = self.__get_srv_name(request.method, request.path)
        if route is None:
            if route in self.__srv_clients.keys():
                del self.__srv_clients[route]
            self.__logger.error(
                'There is no route. ({})'.format(request.path))
            raise web.HTTPNotFound()
        
        http_request = await self.__convert_http_request(request)
        http_request.srv_name = srv_name
        http_request.route = route

        srv_client = self.__get_service_client(srv_name)
        if srv_client is None:
            raise web.HTTPInternalServerError()
        
        http_response: HTTP.Response = await srv_client.call_async(http_request)
        if http_response is None:
            raise web.HTTPInternalServerError()
        
        self.__logger.info('response: {}'.format(http_response))
        
        if http_response.file_path != "":
            return self.__convert_file_response(http_response)
        else:
            return self.__convert_web_response(http_response)
