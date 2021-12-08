#!/usr/bin/env python3
"""
BSD 3-Clause License

Copyright (c) 2020, stimmingexperiments
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import argparse
import logging
import re
import socket
import ssl
import time

import serial


__version__ = '2.0.0'


logger = logging.getLogger(__name__)


class E2B:
    MODE = {
        0: 'Pulse',
        1: 'Bounce',
        2: 'Continuous',
        3: 'Flo',
        4: 'A Split',
        5: 'B Split',
        6: 'Wave',
        7: 'Waterfall',
        8: 'Squeeze',
        9: 'Milk',
        10: 'Throb',
        11: 'Thrust',
        12: 'Cycle',
        13: 'Twist',
        14: 'Random',
        15: 'Step',
        16: 'Training'
    }

    POWER = {
        'L': 'Low',
        'H': 'High',
        'D': 'Dynamic'
    }

    BIAS = {
        0: 'Max',
        1: 'A',
        2: 'B',
        3: 'Average'
    }

    JOINED = {
        0: 'Off',
        1: 'On'
    }

    MAP = {
        0: 'A',
        1: 'B',
        2: 'C'
    }

    WARP = {
        0: 'x1',
        1: 'x2',
        2: 'x4',
        3: 'x8',
        4: 'x16',
        5: 'x32'
    }

    RAMP = {
        0: 'x1',
        1: 'x2',
        2: 'x3',
        3: 'x4'
    }

    TEMPLATE_NEW = (
        'CH-A: {A}%  ' +
        'CH-B: {B}%  ' +
        'MODE: {mode} {C}/{D}  ' +
        'PWR: {power}  ' +
        'BIAS: {bias}  ' +
        'MAP: {map}  ' +
        'RAMP: {ramp}  ' +
        'WARP: {warp}  ' +
        'BAT: {battery}'
    )

    TEMPLATE_OLD = (
        'CH-A: {A}%  ' +
        'CH-B: {B}%  ' +
        'MODE: {mode} {C}/{D}  ' +
        'PWR: {power}  ' +
        'BAT: {battery}'
    )

    def __init__(self, options):
        self.connection = serial.Serial(
            options.port,
            9600,
            timeout=0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        self.options = options
        self.new_format = False
        self.last_status = {}
        self.cmd(b'K', retry=True)

        if self.last_status['joined'] == 'On':
            # Turn off joined mode. Leaving it on creates confusion and allows
            # the remote user to bypass max power limits on channels.
            self.cmd(b'J0')
            if self.last_status['joined'] == 'On':
                # The J commands don't work with all firmwares
                raise IOError('Please turn off A&B Chan Link on the 2B')
            logger.warning('Turned off A&B Chan Link for sanity and safety')

    def _cmd(self, cmd):
        # E-Stim Connect logs "S" when retrieving status so we'll copy it to
        # prevent confusion.
        logger.debug('>SERIAL ' + bytes_to_str(cmd or b'S'))

        self.connection.write(cmd + b'\r')

        # If we read back too quickly we will get an out of date status
        time.sleep(0.15)

        data = None
        while True:
            # If the menu is open a backlog can accumulate so we need to read
            # all available lines
            new_data = self.connection.readline().strip()
            if new_data:
                data = new_data
            else:
                break

        if not data:
            logger.error(
                'No response from 2B, check it\'s connected and the menu ' +
                'isn\'t open'
            )
            return b''

        logger.debug('<SERIAL ' + bytes_to_str(data))

        if data == b'ERR':
            logger.error('Error from 2B')
        else:
            try:
                self.last_status = self.parse_status(data)
            except Exception:
                logger.error('Data from 2B doesn\'t look like a valid status')
                data = b''
            else:
                self.write_status(self.last_status, data)

        return data

    lv_recording = {
        'A': False,
        'B': False
    }
    lv_recording_start_time = {
        'A': 0,
        'B': 0
    }
    lv_floating = {
        'A': False,
        'B': False
    }
    lv_loop = {
        'A': [],
        'B': []
    }
    lv_float = {
        'A': None,
        'B': None
    }

    def cmd_lv_mode(self, cmd, options):
        if not options or not options.lv_mode:
            return

        if cmd.startswith((b'A', b'B')):
            channel = cmd.decode('ascii')[0]
            try:
                value = int(cmd[1:])
            except Exception:
                return b'ERR'

            if value > 100:
                if value == 200:
                    self.lv_recording[channel] = True
                    self.lv_recording_start_time[channel] = \
                        time.perf_counter()
                    self.lv_loop[channel] = []
                elif value == 201:
                    self.lv_recording[channel] = False
                elif value == 202:
                    self.lv_floating[channel] = True
                return b'ACK'
            else:
                if self.lv_recording[channel]:
                    self.lv_loop[channel].append(cmd)
                else:
                    self.lv_loop[channel] = []
                if self.lv_floating[channel] is True:
                    self.lv_float[channel] = cmd
                    self.lv_floating[channel] = False
                else:
                    self.lv_float[channel] = None

        elif not cmd:
            for value in self.lv_float.values():
                if value is not None:
                    return self.cmd(value)

            for channel, values in self.lv_loop.items():
                if values and not self.lv_recording[channel]:
                    return self.cmd(values[round(
                        time.perf_counter() * 4 -
                        self.lv_recording_start_time[channel] * 4
                    ) % len(values)])

    def cmd(self, cmd, options=None, retry=False):
        data = self.cmd_lv_mode(cmd, options)
        if data is not None:
            return data

        while not data:
            data = self._cmd(cmd)

            if not data:
                self.write_status(
                    '2B not responding (cable unplugged or menu open)'
                )

                if retry:
                    logger.error('Automatically retrying in 1 second')
                    time.sleep(1)
                else:
                    # Sending blank data or gibberish bytes causes disconnects
                    # so cover up everything with ERR
                    data = b'ERR'

        return data

    def close(self):
        self.cmd(b'K')
        self.connection.close()
        self.write_status('2B disconnected')

    def parse_status(self, status):
        status = status.split(b':')

        length = len(status)
        if length not in (9, 13):
            raise ValueError()

        self.new_format = length == 13

        if self.new_format:
            values = (
                'battery', 'A', 'B', 'C', 'D', 'mode', 'power', 'bias',
                'joined', 'map', 'warp', 'ramp', 'firmware'
            )
        else:
            values = (
                'battery', 'A', 'B', 'C', 'D', 'mode', 'power', 'joined',
                'firmware'
            )

        result = {}

        for i, key in enumerate(values):
            value = status[i]
            if value.isdigit():
                value = int(value)
            else:
                value = value.decode('ascii')
            result[key] = value

        result['battery'] = (
            'Mains' if result['battery'] >= 750 else
            'Off' if result['battery'] < 320 else
            str(round(100*(result['battery']-320)/422)) + '%'
        )

        for key in ('A', 'B', 'C', 'D'):
            result[key] = result[key] // 2

        if not self.new_format:
            if result['mode'] >= 11:
                result['mode'] += 3
            elif result['mode'] >= 3:
                result['mode'] += 1

        if result['mode'] not in self.MODE:
            logger.error(
                '2B is in an incompatible mode ' +
                '(e.g. stereo, microphone, or tickle modes)'
            )
            raise ValueError()

        for key in ('mode', 'power', 'bias', 'joined', 'map', 'warp', 'ramp'):
            if key in result:
                result[key] = getattr(self, key.upper())[result[key]]

        return result

    def write_status(self, status, raw_status=None):
        if self.options.write_status:
            if isinstance(status, dict):
                if status['battery'] == 'Off':
                    status = '2B connected but turned off'
                elif self.options.write_status_template == '':
                    status = bytes_to_str(raw_status)
                else:
                    if self.options.write_status_template is None:
                        if self.new_format:
                            template = self.TEMPLATE_NEW
                        else:
                            template = self.TEMPLATE_OLD
                    else:
                        template = self.options.write_status_template
                    status = template.format(**status)

            # This isn't atomic so may cause flickering in something like OBS
            with open(self.options.write_status_file, 'w') as f:
                f.write(status)


def bytes_to_str(data):
    try:
        return data.decode('ascii')
    except UnicodeDecodeError:
        # If we receive random bytes this is the best we can do
        return repr(data)


def send_data(sock, data):
    logger.debug('>SERVER ' + bytes_to_str(data))
    sock.sendall(data)


def read_data(sock, expected=None, retry=False):
    data = sock.recv(128)
    data = data.strip()
    logger.debug('<SERVER ' + bytes_to_str(data))
    if expected and not data.startswith(expected):
        if not retry and data == b';200:info:server ping':
            # Sometimes we get a ping response in the middle of the handshake
            # so we should read again
            return read_data(sock, expected, True)
        raise IOError('Unexpected response from server')
    return data


def get_socket():
    for key in ('socks4', 'socks5', 'http'):
        proxy = getattr(options, key + '_proxy')
        if proxy:
            try:
                import socks
            except ImportError:
                raise IOError('You must install PySocks to use a proxy')
            sock = socks.socksocket()
            proxy = proxy.split(':')
            if len(proxy) == 2:
                port = int(proxy[1])
            else:
                port = None
            sock.set_proxy(getattr(socks, key.upper()), proxy[0], port)
            break
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    sock.settimeout(3)
    sock.connect(('e-stim.online', 57846))
    sock = ssl.wrap_socket(sock)

    return sock


def handshake(e2b, sock, key=None):
    read_data(sock, b';200:info:')

    app_name = b'stimmingexperiments ' + __version__.encode('ascii')

    if key is None:
        send_data(
            sock, b';getkey:' + app_name
        )
        key = read_data(sock, b';203:key=').split(b'=')[-1]
    else:
        key = key.encode('ascii')

    data = b';wait=' + key + b';hello=' + app_name
    send_data(sock, data)
    handle_response(read_data(sock, (b';300:waiting', b';201:connected')))

    if options.nickname:
        nickname = ''.join([
            c if c.isalnum() else '_' for c in options.nickname
        ])[:30]
        send_data(sock, b';setnick=' + nickname.encode('ascii'))
        read_data(sock, b';200:nickname set')

    logger.info(
        'Link: https://e-stim.online/connect/?' + key.decode('ascii')
    )

    send_data(sock, e2b.cmd(b''))


def handle_response(data):
    if not data.startswith(b';'):
        return data
    elif data == b';200:info:server ping':
        pass
    elif data == b';201:connected':
        logger.info('Remote user connected')
    elif data == b';202:disconnected':
        logger.warning('Remote user disconnected')
        return False
    elif data.startswith(b';210'):
        logger.warning(
            'Broadcast message: ' + bytes_to_str(data.split(b':', 2)[2])
        )
    elif data == b';300:waiting':
        pass
    elif data == b';400:peer disconnected':
        logger.warning('Remote user unexpectedly disconnected')
        return False
    else:
        logger.warning(
            'Unhandled response type from server: ' + bytes_to_str(data)
        )


def cmd_timeout(last_cmd, options):
    if options.switch_off_after_timeout:
        if (
            (time.time() - last_cmd) / 60 >=
            options.switch_off_after_timeout
        ):
            logger.error('Timeout has expired since last command')
            return True


def filter_blocked(cmd, e2b, options):
    cmd = cmd.upper()

    if not re.match(b'^[A-Z]([+-]|[0-9]{0,3})?$', cmd):
        logger.error('Bad command format')
    elif cmd[0:1] == b'J':
        logger.critical(
            'Remote user is attempting to change A&B Chan Link setting ' +
            'which may be used to bypass a max channel level setting'
        )
    elif options.prevent_power_level_change and cmd[0:1] in (
        b'H', b'L', b'D'
    ):
        logger.error('Prevented power level change')
    elif options.prevent_program_mode_change and cmd.startswith(b'M'):
        logger.error('Prevented program mode change')
    else:
        for ch in ('A', 'B'):
            max_ = getattr(options, 'max_level_' + ch.lower())
            if max_ and cmd.startswith(ch.encode('ascii')):
                remaining = max_ - e2b.last_status.get(ch, 0)
                if remaining <= 0:
                    if (
                        cmd[1:2] == b'+'
                    ) or (
                        cmd[1:2] not in (b'-', b'+') and
                        int(cmd[1:]) > max_
                    ):
                        logger.error(
                            'Prevented power level increase on channel ' + ch
                        )
                        return
                if cmd[1:2] not in (b'-', b'+') and int(cmd[1:]) > max_:
                    # If going past the limit max out to what's allowed
                    return ('%s%s' % (ch, max_)).encode('ascii')
        return cmd


def run(options):
    e2b = None
    sock = None

    try:
        e2b = E2B(options)
        sock = get_socket()

        handshake(e2b, sock, options.reconnect_key)

        last_cmd = time.time()

        # I didn't implement ;ping so we also rely on the control monitoring to
        # keep the connection alive
        sock_timeout = max(1, min(options.monitor_controls, 10))
        if options.lv_mode:
            sock_timeout = 0.1

        while True:
            cmds = []

            try:
                # There is a risk of a flood of A+/B+ commands if the remote
                # user spams buttons during a period of network or serial
                # latency. This code is a strange way to handle it but the
                # alternative is to make the whole script async or threaded
                # which adds complexity elsewhere.
                sock.settimeout(sock_timeout)
                max_reads = 100  # Avoid an infinite loop
                while max_reads:
                    data = handle_response(read_data(sock))
                    if data is False:
                        # If remote user disconnects then end recording to
                        # avoid a leak if they reconnect
                        for c in (b'A201', b'B201'):
                            e2b.cmd_lv_mode(c, options)
                    elif data:
                        cmds.append(data)
                    sock.settimeout(0)
                    max_reads -= 1
            except (socket.timeout, ssl.SSLWantReadError):
                if cmd_timeout(last_cmd, options):
                    break

            if cmds:
                last_cmd = time.time()

                if not options.lv_mode and len(cmds) > 1:
                    logger.warning(
                        'Flood protection ignoring %s commands' % len(cmds)
                    )
                    cmds = []

            data = None

            for cmd in cmds:
                if cmd is not None:
                    cmd = filter_blocked(cmd, e2b, options)
                    if cmd is None:
                        send_data(sock, b';701:command blocked')
                    else:
                        data = e2b.cmd(cmd, options)

            if data is None:
                data = e2b.cmd(b'', options)

            send_data(sock, data)
    except (IOError, OSError) as e:
        logger.error(e)
        if sock:
            logger.error('Disconnected from e-stim.online')
    except KeyboardInterrupt:
        pass
    finally:
        if sock:
            try:
                send_data(sock, b';quit')
            except (IOError, OSError):
                pass
            sock.close()
        if e2b:
            e2b.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('port', help='The serial port your 2B is connected to')
    parser.add_argument(
        'nickname', nargs='?', default='', help='Your nickname'
    )

    parser.add_argument(
        '--switch-off-after-timeout', '-t', type=int,
        help='Switch off after timeout (minutes)'
    )
    parser.add_argument(
        '--prevent-power-level-change', '-pp', action='store_true',
        help='Prevent power level change'
    )
    parser.add_argument(
        '--prevent-program-mode-change', '-pm', action='store_true',
        help='Prevent program mode change'
    )
    parser.add_argument(
        '--max-level-a', '-ma', type=int,
        help='Set maximum level for channel A'
    )
    parser.add_argument(
        '--max-level-b', '-mb', type=int,
        help='Set maximum level for channel B'
    )
    parser.add_argument(
        '--monitor-controls', '-mc', type=int, default=5,
        help='Monitor 2B controls every 1-10 seconds'
    )

    parser.add_argument(
        '--socks4-proxy', '-s4p', help='SOCKS4 proxy'
    )
    parser.add_argument(
        '--socks5-proxy', '-s5p', help='SOCKS5 proxy'
    )
    parser.add_argument(
        '--http-proxy', '-hp', help='HTTP proxy'
    )

    parser.add_argument(
        '--write-status', '-ws', action='store_true',
        help='Write current status to a file'
    )
    parser.add_argument(
        '--write-status-file', '-wsf', default='e2bconnect.txt',
        help='File to write status to (default is e2bconnect.txt)'
    )
    parser.add_argument(
        '--write-status-template', '-wst', nargs='?', default='',
        help=(
            'Customise the output format of the status ' +
            '(a blank value uses the default template)'
        )
    )

    parser.add_argument(
        '--reconnect-key', '-r',
        help='Reconnect to a previously acquired key'
    )
    parser.add_argument(
        '--lv-mode', '-lv', action='store_true',
        help='Required for Lv style mode'
    )

    parser.add_argument(
        '--debug', '-d', action='store_true',
        help='Change log level to debug'
    )

    options = parser.parse_args()

    colorlog = False
    log_format = '%(asctime)s %(levelname)s %(message)s'
    try:
        import colorlog
        log_format = '%(log_color)s' + log_format
        handler_cls = colorlog.StreamHandler
        formatter_cls = colorlog.ColoredFormatter
    except ImportError:
        handler_cls = logging.StreamHandler
        formatter_cls = logging.Formatter

    root_logger = logging.getLogger()
    handler = handler_cls()
    handler.setFormatter(formatter_cls(
        log_format, datefmt='%Y-%m-%d %H:%M:%S'
    ))
    root_logger.addHandler(handler)

    if options.debug:
        root_logger.setLevel(logging.DEBUG)
    else:
        root_logger.setLevel(logging.INFO)

    if not colorlog:
        logger.warning(
            'It\'s recommended to install colorlog for colored log output ' +
            'as it will make it easier to notice events from a distance'
        )

    run(options)
