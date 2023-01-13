import asyncio
import os
import os.path
import pathlib
import secrets
import socket
import ssl
from typing import NoReturn
from aiohttp import web

import rclpy
import rclpy.logging

from ros2web.verb import VerbExtension
from ros2web.api.handler import HandlerNode
from ros2web.api.ros_executor import ROSExecutor
from ros2web.utilities import get_ip_address


@web.middleware
async def token_auth(request: web.Request, handler):
    _token = request.app["token"]

    search_params = request.rel_url.query
    token_param = search_params.get("token")

    if token_param:
        if _token != token_param:
            raise web.HTTPUnauthorized()
        else:
            exc = web.HTTPFound(location=request.path)
            exc.set_cookie("AUTH", _token)
            raise exc

    token_header = request.headers.get("token")
    if token_header:
        token = token_header
    else:
        token = request.cookies.get("AUTH")

    if _token != token:
        raise web.HTTPUnauthorized()

    response = await handler(request)
    return response


class ServerVerb(VerbExtension):

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--bind',
            help='Specify alternate bind address [default: all interfaces]')
        parser.add_argument(
            '--port',
            default='8080',
            help='Specify alternate port [default: 8080]')

        parser.add_argument(
            '--destination-directory',
            default=os.path.join(os.path.expanduser('~'), '.ros2web'),
            help='Directory where to create the config directory')

        parser.add_argument(
            '--node-name',
            default='ros2web',
            help='Node name [default: ros2web]')

        parser.add_argument(
            '--namespace',
            default=None,
            help='namespace [default: None]')

        parser.add_argument(
            '--log-level',
            default='ERROR',
            choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
            help='Log Level [default: ERROR]')

        parser.add_argument(
            '--non-auth',
            action='store_true',
            help='Token is not used.')

    def main(self, *, args):
        self.__port = int(args.port)
        self.__directory = args.destination_directory
        self.__host = args.bind
        self.__log_level = args.log_level
        self.__node_name = args.node_name
        self.__namespace = args.namespace
        self.__non_auth = args.non_auth

        self.__logger = rclpy.logging.get_logger('ServerVerb')

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        self.__token = None if self.__non_auth is True else secrets.token_hex(16)
        ip_address = self.__host
        if self.__host is None:
            ip_address = get_ip_address()

        if self.available_port(ip_address, self.__port):
            self.__ros_executor = ROSExecutor(node_name=self.__node_name,
                                            namespace=self.__namespace,
                                            args=['--ros-args', '--log-level',
                                                    self.__log_level],
                                            loop=loop)

            self.handler_node = HandlerNode(directory=self.__directory,
                                            ip_address=ip_address, port=self.__port, token=self.__token,
                                            loop=loop)

            app = self.init_web_app(token=self.__token)

            here = pathlib.Path(self.__directory)
            ssl_cert = here / "ssl" / "server.crt"
            ssl_key = here / "ssl" / "server.key"
            ssl_context = None
            if ssl_cert.exists() and ssl_key.exists():
                ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
                ssl_context.load_cert_chain(str(ssl_cert), str(ssl_key))

            web.run_app(app, host=self.__host, port=self.__port,
                        loop=loop, ssl_context=ssl_context, print=self.print)
        else:
            print("Address('{}', {}) already in use".format(ip_address, self.__port))
            
    def print(self, str):
        print(str)
        print("token={}\n".format(self.__token))

    def available_port(self, ip_address, port):
        is_available = False
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result = sock.connect_ex((ip_address, port))
        if result != 0:
            is_available = True
        sock.close()
        return is_available

    def init_web_app(self, *, token: str) -> web.Application:

        if token is None:
            app = web.Application()
        else:
            app = web.Application(middlewares=[token_auth])
            app['token'] = token

        app.router.add_get("/{path:.*}", self.handler_node.web_handler)
        app.router.add_post("/{path:.*}", self.handler_node.web_handler)
        app.router.add_put("/{path:.*}", self.handler_node.web_handler)
        app.router.add_delete("/{path:.*}", self.handler_node.web_handler)

        app.on_startup.append(self.on_startup)
        app.on_shutdown.append(self.on_shutdown)
        app.on_cleanup.append(self.on_cleanup)

        return app

    async def root_handler(self, request: web.Request) -> NoReturn:
        raise web.HTTPNotFound()

    async def on_startup(self, app: web.Application) -> None:
        self.__logger.info('on_startup')
        self.__ros_executor.start()
        await self.handler_node.startup(ros_node=self.__ros_executor.node)

    async def on_shutdown(self, app: web.Application) -> None:
        self.__logger.info('on_shutdown')
        await self.handler_node.shutdown()
        self.__ros_executor.shutdown()

    async def on_cleanup(self, app: web.Application) -> None:
        self.__logger.info('on_cleanup')
        await asyncio.sleep(0.2)
        # await self.handler_node.cleanup()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    cli = ServerVerb()
    cli.add_arguments(parser, None)
    args = parser.parse_args()
    cli.main(args=args)
