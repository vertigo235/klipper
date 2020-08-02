# Klippy WebHooks registration and server connection
#
# Copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license
import logging
import socket
import errno
import json
import homing

SOCKET_LOCATION = "/tmp/moonraker"

# Json decodes strings as unicode types in Python 2.x.  This doesn't
# play well with some parts of Klipper (particuarly displays), so we
# need to create an object hook. This solution borrowed from:
#
# https://stackoverflow.com/questions/956867/
#
def byteify(data, ignore_dicts=False):
    if isinstance(data, unicode):
        return data.encode('utf-8')
    if isinstance(data, list):
        return [byteify(i, True) for i in data]
    if isinstance(data, dict) and not ignore_dicts:
        return {byteify(k, True): byteify(v, True)
                for k, v in data.items()}
    return data

def json_loads_byteified(data):
    return byteify(
        json.loads(data, object_hook=byteify), True)

class WebRequestError(homing.CommandError):
    def __init__(self, message,):
        Exception.__init__(self, message)

    def to_dict(self):
        return {
            'error': 'WebRequestError',
            'message': self.message}

class Sentinel:
    pass

class WebRequest:
    error = WebRequestError
    def __init__(self, base_request):
        self.id = base_request['id']
        self.path = base_request['path']
        self.method = base_request['method']
        self.args = base_request['args']
        self.response = None

    def get(self, item, default=Sentinel):
        if item not in self.args:
            if default == Sentinel:
                raise WebRequestError("Invalid Argument [%s]" % item)
            return default
        return self.args[item]

    def get_int(self, item):
        return int(self.get(item))

    def get_float(self, item):
        return float(self.get(item))

    def get_args(self):
        return self.args

    def get_path(self):
        return self.path

    def get_method(self):
        return self.method

    def set_error(self, error):
        self.response = error.to_dict()

    def send(self, data):
        if self.response is not None:
            raise WebRequestError("Multiple calls to send not allowed")
        self.response = data

    def finish(self):
        if self.response is None:
            # No error was set and the user never executed
            # send, default response is "ok"
            self.response = "ok"
        return {"request_id": self.id, "response": self.response}

class ServerConnection:
    def __init__(self, webhooks, printer):
        self.printer = printer
        self.webhooks = webhooks
        self.reactor = printer.get_reactor()

        # Klippy Connection
        self.fd = self.fd_handle = self.mutex = None
        self.is_server_connected = False
        self.partial_data = ""
        is_fileinput = (printer.get_start_args().get('debuginput')
                        is not None)
        if is_fileinput:
            # Do not try to connect in klippy batch mode
            return
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.setblocking(0)
        try:
            self.socket.connect(SOCKET_LOCATION)
        except socket.error:
            logging.debug(
                "ServerConnection: Moonraker server not detected")
            return
        logging.debug("ServerConnection: Moonraker connection established")
        self.is_server_connected = True
        self.fd = self.socket.fileno()
        self.fd_handle = self.reactor.register_fd(
            self.fd, self.process_received)
        self.mutex = self.reactor.mutex()
        printer.register_event_handler('klippy:disconnect', self.close_socket)

    def close_socket(self):
        if self.is_server_connected:
            logging.info("ServerConnection: lost connection to Moonraker")
            self.is_server_connected = False
            self.reactor.unregister_fd(self.fd_handle)
            try:
                self.socket.close()
            except socket.error:
                pass

    def is_connected(self):
        return self.is_server_connected

    def process_received(self, eventtime):
        try:
            data = self.socket.recv(4096)
        except socket.error as e:
            # If bad file descriptor allow connection to be
            # closed by the data check
            if e.errno == errno.EBADF:
                data = ''
            else:
                return
        if data == '':
            # Socket Closed
            self.close_socket()
            return
        requests = data.split('\x03')
        requests[0] = self.partial_data + requests[0]
        self.partial_data = requests.pop()
        for req in requests:
            logging.debug(
                "ServerConnection: Request received: %s" % (req))
            try:
                web_request = WebRequest(json_loads_byteified(req))
            except Exception:
                logging.exception(
                    "ServerConnection: Error decoding Server Request %s"
                    % (req))
                continue
            self.reactor.register_callback(
                lambda e, s=self, wr=web_request: s._process_request(wr))

    def _process_request(self, web_request):
        try:
            func = self.webhooks.get_callback(
                web_request.get_path())
            func(web_request)
        except homing.CommandError as e:
            web_request.set_error(WebRequestError(e.message))
        except Exception as e:
            msg = "Internal Error on WebRequest: %s" % (web_request.get_path())
            logging.exception(msg)
            web_request.set_error(WebRequestError(e.message))
            self.printer.invoke_shutdown(msg)
        result = web_request.finish()
        logging.debug(
            "ServerConnection: Sending response - %s" % (str(result)))
        self.send({'method': "response", 'params': result})

    def send(self, data):
        if not self.is_server_connected:
            return
        with self.mutex:
            retries = 10
            data = json.dumps(data) + "\x03"
            while data:
                try:
                    sent = self.socket.send(data)
                except socket.error as e:
                    if e.errno == errno.EBADF or e.errno == errno.EPIPE \
                            or not retries:
                        sent = 0
                    else:
                        retries -= 1
                        waketime = self.reactor.monotonic() + .001
                        self.reactor.pause(waketime)
                        continue
                retries = 10
                if sent > 0:
                    data = data[sent:]
                else:
                    logging.info(
                        "ServerConnection: Error sending server data,"
                        " closing socket")
                    self.close_socket()
                    break

class WebHooks:
    def __init__(self, printer):
        self.printer = printer
        self._endpoints = {"list_endpoints": self._handle_list_endpoints}
        self._static_paths = []
        self.register_endpoint("info", self._handle_info_request)
        self.register_endpoint("emergency_stop", self._handle_estop_request)
        start_args = printer.get_start_args()
        log_file = start_args.get('log_file')
        if log_file is not None:
            self.register_static_path("klippy.log", log_file)
        self.sconn = ServerConnection(self, printer)

    def register_endpoint(self, path, callback):
        if path in self._endpoints:
            raise WebRequestError("Path already registered to an endpoint")
        self._endpoints[path] = callback


    def register_static_path(self, resource_id, file_path):
        static_path_info = {
            'resource_id': resource_id, 'file_path': file_path}
        self._static_paths.append(static_path_info)

    def _handle_list_endpoints(self, web_request):
        web_request.send({
            'hooks': self._endpoints.keys(),
            'static_paths': self._static_paths})

    def _handle_info_request(self, web_request):
        if web_request.get_method() != 'GET':
            raise web_request.error("Invalid Request Method")
        start_args = self.printer.get_start_args()
        state_message, msg_type = self.printer.get_state_message()
        version = start_args['software_version']
        cpu_info = start_args['cpu_info']
        error = msg_type == "error"
        web_request.send(
            {'cpu': cpu_info, 'version': version,
             'hostname': socket.gethostname(),
             'is_ready': msg_type == "ready",
             'error_detected': error,
             'message': state_message})

    def _handle_estop_request(self, web_request):
        if web_request.get_method() != 'POST':
            raise web_request.error("Invalid Request Method")
        gcode = self.printer.lookup_object('gcode')
        gcode.cmd_M112(None)

    def get_connection(self):
        return self.sconn

    def get_callback(self, path):
        cb = self._endpoints.get(path, None)
        if cb is None:
            msg = "webhooks: No registered callback for path '%s'" % (path)
            logging.info(msg)
            raise WebRequestError(msg)
        return cb
