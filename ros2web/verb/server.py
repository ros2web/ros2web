from typing import NoReturn

import os
import os.path
import asyncio
import secrets
from aiohttp import web

import launch.logging
from ros2web.verb import VerbExtension

from ..api.handler import HandlerNode
from ..utilities import get_ip_address

logger = launch.logging.get_logger('server')


async def root_handler(request: web.Request) -> NoReturn:
    raise web.HTTPNotFound()


async def on_startup(app: web.Application) -> None:
    logger.debug('on_startup')
    handler_node: HandlerNode = app['handler_node']
    await handler_node.startup()
    

async def on_shutdown(app: web.Application) -> None:
    logger.debug('on_shutdown')
    handler_node: HandlerNode = app['handler_node']
    await handler_node.shutdown()
    


async def on_cleanup(app: web.Application) -> None:
    logger.debug('on_cleanup')
    handler_node: HandlerNode = app['handler_node']
    await handler_node.cleanup()
    

@web.middleware
async def token_auth(request: web.Request, handler):
    response = await handler(request)
    return response


def init_web_app(*, handler_node: HandlerNode) -> web.Application:
    # app = web.Application(middlewares=[token_auth])
    app = web.Application()
    app['handler_node'] = handler_node

    # router
    app.router.add_get("/{path:.*}", handler_node.web_handler)
    app.router.add_post("/{path:.*}", handler_node.web_handler)
    app.router.add_put("/{path:.*}", handler_node.web_handler)
    app.router.add_delete("/{path:.*}", handler_node.web_handler)
    
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    app.on_cleanup.append(on_cleanup)

    return app

def exception_handler(loop, context):
    print(context)
    
class ServerVerb(VerbExtension):
        
    """Start the server."""
    async def __start_server(self, *, loop: asyncio.AbstractEventLoop):
        try:
            token = secrets.token_hex(16)
            ip_address = self.__host
            if self.__host is None:
                ip_address = get_ip_address()
            
            node_name = 'ros2web'
            handler_node = HandlerNode(node_name=node_name, directory=self.__directory,
                                     ip_address=ip_address, port=self.__port, token=token,
                                     loop=loop)
            
            runner = web.AppRunner(init_web_app(handler_node=handler_node))
            await runner.setup()

            site = web.TCPSite(runner, self.__host, self.__port)
            await site.start()
            
            print("(Press CTRL+C to quit)\n")
            
            completed_future = loop.create_future()
            await completed_future
        except asyncio.CancelledError:
            pass
        finally:
            await runner.cleanup()
            pass
            

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
            default=os.path.expanduser('~'),
            help='Directory where to create the config directory')

    def main(self, *, args):
        self.__port = int(args.port)
        self.__directory = args.destination_directory
        self.__host = args.bind
        
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.set_exception_handler(exception_handler)
        
        try:
            task = loop.create_task(self.__start_server(loop=loop))
            loop.run_forever()
        except KeyboardInterrupt:
            task.cancel()
            loop.run_until_complete(task)
        finally:
            loop.close()
        
        
