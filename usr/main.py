import utime
import _thread
import gc
import net
import dataCall
import checkNet
import ntptime
import modem
import usocket
from machine import Pin, UART
from misc import Power


class WatchDog:
    def __init__(self, max_count=6):
        self.max_count = max_count
        self.count = max_count
        self.tid = None

    def feed(self):
        self.count = self.max_count

    def _run(self):
        while True:
            if self.count == 0:
                print('Watchdog: triggering reset')
                Power.powerRestart()
            else:
                self.count -= 1
            utime.sleep(10)

    def start(self):
        if not self.tid or not _thread.threadIsRunning(self.tid):
            _thread.stack_size(0x1000)
            self.tid = _thread.start_new_thread(self._run, ())

    def stop(self):
        if self.tid:
            try:
                _thread.stop_thread(self.tid)
            except:
                pass
            self.tid = None

from usr.config import Config
from usr.led_controller import Leds, Led
from usr.gps_controller import GPSController
from usr.wifi_scanner import WiFiScanner
from usr.battery import BatteryMonitor
from usr.sms_handler import SMSHandler
from usr.data_buffer import DataBuffer
from usr.gt06_protocol import GT06Protocol
from usr.http_protocol import HTTPProtocol


GNSS_PORT = UART.UART2
GNSS_PIN = Pin.GPIO10

LED_RED = Pin.GPIO15
LED_BLUE = Pin.GPIO16
LED_YELLOW = Pin.GPIO17


class GPSTracker:
	"""Main GPS Tracker class"""

	def __init__(self):
		print('Initializing GPS Tracker...')
		self.state_lock = _thread.allocate_lock()
		self.config = Config()
		self.leds = Leds(red_pin=LED_RED, blue_pin=LED_BLUE, yellow_pin=LED_YELLOW)
		self.leds.set_battery_status(Led.MODE_ON)
		self.battery = BatteryMonitor()
		self.gps = GPSController(GNSS_PORT, GNSS_PIN)
		self.wifi_scanner = WiFiScanner()
		self.sms_handler = SMSHandler(self.config, self._config_callback)
		self.data_buffer = DataBuffer()
		self.protocol = None
		self._init_protocol()
		self.running = True
		self.sleep_mode = False
		self.reconfig_needed = False
		self.last_movement_time = utime.time()
		self.last_location = None
		self.connected = False
		self.gps_available = False
		self.ntp_synced = False
		self.last_rtc_sync = 0
		self.wake_event = False
		self.watchdog = WatchDog(12)
		self.watchdog.start()
		_thread.start_new_thread(self._main_loop, ())
		_thread.start_new_thread(self._battery_monitor_loop, ())
		print('GPS Tracker initialized')

	def _init_protocol(self):
		"""Initialize communication protocol"""
		if self.protocol:
			self.protocol.disconnect()
		server = self.config.get('server')
		if server and server['host'] and server['port']:
			protocol_type = server['protocol'].upper()
			if protocol_type == 'GT06':
				self.protocol = GT06Protocol(server['host'], server['port'], self.leds)
			elif protocol_type == 'HTTP':
				self.protocol = HTTPProtocol(server['host'], server['port'], server.get('path', '/api/location'), self.leds)
			else:
				print('Unknown protocol:', protocol_type)
				self.protocol = None
		else:
			self.protocol = None
			print('Server not configured')

	def _config_callback(self, event, *args):
		"""Callback on configuration change"""
		with self.state_lock:
			self.wake_event = True
		if event in ['apn_changed', 'server_changed']:
			print('Configuration changed, scheduling reinit...')
			with self.state_lock:
				self.reconfig_needed = True
		elif event == 'interval_changed':
			print('Update interval changed')
		elif event == 'wifi_server_changed':
			print('WiFi location server changed')
		elif event == 'get_status':
			return self._get_status()
		elif event == 'poweroff':
			self._poweroff()
		elif event == 'reset':
			self._reset()

	def _init_network(self):
		"""Initialize network connection"""
		try:
			checkNet.waitNetworkReady(30)
			apn_config = self.config.get('apn')
			if apn_config['name']:
				dataCall.setApn(1, 0, apn_config['name'], apn_config['user'], apn_config['password'], 0)
			dataCall.setCallback(self._datacall_callback)
			ret = dataCall.activate(1)
			print('Network initialized, PDP active:', ret == 0)
			if ret == 0 and not self.ntp_synced:
				self._sync_ntp()
			return ret == 0
		except Exception as e:
			print('Network init error:', e)
			return False

	def _datacall_callback(self, args):
		"""Callback on PDP context state change"""
		pdp_id = args[0]
		status = args[1]
		print('PDP context', pdp_id, 'status:', status)
		if status == 1:
			print('Network connected')
			if not self.ntp_synced:
				self._sync_ntp()
		else:
			print('Network disconnected')

	def _sync_ntp(self):
		"""Sync time via NTP"""
		try:
			print('Syncing time via NTP...')
			ntptime.host = 'pool.ntp.org'
			ntptime.settime()
			self.ntp_synced = True
			print('NTP time synced')
		except Exception as e:
			print('NTP sync error:', e)

	def _main_loop(self):
		"""Main tracker loop"""
		print('[MAIN] _main_loop started, initializing network...')
		self._init_network()
		print('[MAIN] Network init done, protocol={}'.format(self.protocol))
		if self.protocol:
			print('[MAIN] Calling protocol.connect()...')
			self.protocol.connect()
			print('[MAIN] protocol.connect() returned, connected={}'.format(self.protocol.connected))
		self.gps.enable()
		self.leds.set_gps_status(Led.MODE_BLINK_1HZ)
		update_interval = self.config.get('update_interval', 10)
		last_update = 0
		while self.running:
			try:
				self.watchdog.feed()
				current_time = utime.time()
				with self.state_lock:
					if self.reconfig_needed:
						self.reconfig_needed = False
						print('Processing deferred reconfiguration...')
						self._init_network()
						self._init_protocol()
				if self.sleep_mode:
					print('[SLEEP] Sleeping, waiting for wake event...')
					utime.sleep(30)
					with self.state_lock:
						if self.wake_event:
							self.wake_event = False
							print('[SLEEP] Wake event received')
							self._exit_sleep_mode()
					continue
				if self._check_sleep_mode():
					print('[SLEEP] Entering sleep (idle timeout)')
					self._enter_sleep_mode()
					continue
				gps_valid = self.gps.is_valid()
				if gps_valid:
					self.leds.set_gps_status(Led.MODE_ON)
					with self.state_lock:
						self.gps_available = True
					if current_time - self.last_rtc_sync >= 3600:
						self.gps.sync_rtc()
						self.last_rtc_sync = current_time
				else:
					self.leds.set_gps_status(Led.MODE_BLINK_1HZ)
					with self.state_lock:
						self.gps_available = False
				if current_time - last_update >= update_interval:
					print('[LOOP] Send interval reached ({}s)'.format(update_interval))
					self._send_location_data()
					last_update = current_time
				if self.connected and self.data_buffer.size() > 0:
					self._send_buffered_data()
				utime.sleep(1)
			except Exception as e:
				print('Main loop error:', e)
				utime.sleep(5)

	def _send_or_buffer(self, data):
		"""Send data via protocol or buffer it"""
		source = data.get('source', '?')
		if source == 'wifi' and isinstance(self.protocol, GT06Protocol):
			self._send_wifi_via_osmand(data)
			return
		if self.protocol:
			self.leds.set_network_status(Led.MODE_PULSE)
			self.leds.network_data_start()
			success = self.protocol.send_location(data)
			self.leds.network_data_stop()
			with self.state_lock:
				self.connected = success
			if success:
				print('[TX] {} sent OK'.format(source))
			elif self.config.get('buffer_enabled'):
				if self.data_buffer.add(data):
					print('[TX] {} send FAIL, buffered (size={})'.format(source, self.data_buffer.size()))
				else:
					print('[TX] {} send FAIL, buffer FULL'.format(source))
		elif self.config.get('buffer_enabled'):
			self.data_buffer.add(data)
			print('[TX] No server, {} buffered (size={})'.format(source, self.data_buffer.size()))

	def _send_wifi_via_osmand(self, data):
		"""Send WiFi scan to Traccar via Osmand HTTP endpoint (port 5055)"""
		wifi_networks = data.get('wifi_networks', [])
		if not wifi_networks:
			return
		with self.state_lock:
			last_pos = self.last_location
		lat = last_pos['latitude'] if last_pos and last_pos.get('valid') else None
		lon = last_pos['longitude'] if last_pos and last_pos.get('valid') else None
		try:
			server = self.config.get('server')
			host = server['host'] if server and server['host'] else 'localhost'
			imei = modem.getDevImei()
			ts = int(utime.time() * 1000)
			batt = data.get('battery', 0)
			params = 'id={}&timestamp={}&batt={}'.format(imei, ts, batt)
			if lat is not None and lon is not None:
				params += '&lat={}&lon={}&hdop=99'.format(lat, lon)
			for ap in wifi_networks:
				params += '&wifi={},{}'.format(ap['mac'], ap['signal'])
			request = 'GET /?' + params + ' HTTP/1.1\r\nHost: ' + host + ':5055\r\nConnection: close\r\n\r\n'
			self.leds.set_network_status(Led.MODE_BLINK_CONNECT)
			sock = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
			sock.settimeout(10)
			addr = usocket.getaddrinfo(host, 5055)[0][-1]
			sock.connect(addr)
			sock.send(request.encode())
			while sock.recv(1024):
				pass
			sock.close()
			with self.state_lock:
				self.connected = True
			self.leds.set_network_status(Led.MODE_PULSE)
			print('[OSMAND] WiFi scan sent: {} APs'.format(len(wifi_networks)))
		except Exception as e:
			print('[OSMAND] Error:', e)
			self.leds.set_network_status(Led.MODE_OFF)

	def _send_location_data(self):
		"""Send location data"""
		try:
			location = None
			wifi_networks = []
			if self.gps_available:
				location = self.gps.get_location()

			# WiFi fallback when GPS unavailable
			if not location or not location.get('valid'):
				wifi_enabled = self.config.get('wifi_location_enabled', False)
				if wifi_enabled:
					print('[LOC] GPS invalid, scanning WiFi...')
					wifi_networks = self.wifi_scanner.scan_networks()
					if wifi_networks:
						print('[LOC] WiFi fallback: {} APs found'.format(len(wifi_networks)))
						data = {
							'timestamp': utime.time(),
							'battery': self.battery.get_percentage(),
							'charging': self.battery.is_charging,
							'source': 'wifi',
							'wifi_networks': wifi_networks
						}
						self._send_or_buffer(data)
						return

			if not location:
				print('[LOC] No location data available (GPS+WiFi failed)')
				return

			# GPS data available (with optional WiFi augmentation)
			if self._detect_movement(location):
				with self.state_lock:
					self.last_movement_time = utime.time()
				print('[LOC] Movement detected, idle timer reset')

			data = {'timestamp': utime.time(), 'latitude': location['latitude'], 'longitude': location['longitude'], 'altitude': location.get('altitude', 0.0), 'speed': location.get('speed', 0.0), 'course': location.get('course', 0.0), 'satellites': location.get(
				'satellites', 0), 'battery': self.battery.get_percentage(), 'charging': self.battery.is_charging, 'valid': location.get('valid', False), 'source': location.get('source', 'gps'), 'accuracy': location.get('accuracy', 0)}
			if wifi_networks:
				data['wifi_networks'] = wifi_networks

			self._send_or_buffer(data)

			with self.state_lock:
				self.last_location = location
		except Exception as e:
			print('Send location error:', e)

	def _send_buffered_data(self):
		"""Send buffered data"""
		try:
			sent_count = 0
			for data in self.data_buffer.get_points():
				self.leds.network_data_start()
				success = self.protocol.send_location(data)
				self.leds.network_data_stop()
				if success:
					sent_count += 1
				else:
					print('Failed to send buffered data, stopping')
					break
				utime.sleep_ms(100)
			if sent_count > 0:
				self.data_buffer.remove(sent_count)
				print('Sent {} buffered records'.format(sent_count))
		except Exception as e:
			print('Send buffered data error:', e)

	def _detect_movement(self, location):
		"""Detect movement based on location change"""
		if not self.last_location:
			return True
		# gnss.getSpeed() returns km/h — 1 km/h ≈ 0.28 m/s
		if location.get('speed', 0) > 3.0:
			return True
		lat_diff = abs(location['latitude'] - self.last_location['latitude'])
		lon_diff = abs(location['longitude'] - self.last_location['longitude'])
		if lat_diff > 0.0001 or lon_diff > 0.0001:
			return True
		return False

	def _check_sleep_mode(self):
		"""Check if should enter sleep mode"""
		with self.state_lock:
			if self.sleep_mode:
				return False
			idle_time = utime.time() - self.last_movement_time
		sleep_timeout = self.config.get('sleep_timeout', 1800)
		if idle_time >= sleep_timeout:
			print('[SLEEP] Idle {}s >= timeout {}s, entering sleep'.format(idle_time, sleep_timeout))
			return True
		return False

	def _enter_sleep_mode(self):
		"""Enter sleep mode"""
		with self.state_lock:
			if self.sleep_mode:
				return
			self.sleep_mode = True
		print('Entering sleep mode')
		self.gps.disable()
		self.leds.set_gps_status(Led.MODE_OFF)
		self.wifi_scanner.disable()
		self.leds.set_network_status(Led.MODE_OFF)
		self.leds.set_battery_status(Led.MODE_BLINK_SLOW)
		if self.protocol:
			self.protocol.disconnect()
		print('Sleep mode active')

	def _exit_sleep_mode(self):
		"""Exit sleep mode"""
		with self.state_lock:
			if not self.sleep_mode:
				return
			self.sleep_mode = False
			self.last_movement_time = utime.time()
		print('Exiting sleep mode')
		self.gps.enable()
		self.leds.set_gps_status(Led.MODE_BLINK_1HZ)
		self._update_battery_led()
		if self.protocol:
			self.protocol.connect()
		print('Sleep mode exited')

	def _battery_monitor_loop(self):
		"""Battery monitoring loop"""
		while self.running:
			try:
				self.battery.update()
				with self.state_lock:
					sleeping = self.sleep_mode
				if not sleeping:
					self._update_battery_led()
				utime.sleep(5)
			except Exception as e:
				print('Battery monitor error:', e)
				utime.sleep(10)

	def _update_battery_led(self):
		"""Update battery status LED"""
		if self.battery.is_charging:
			self.leds.set_battery_status(Led.MODE_BLINK_1HZ)
		elif self.battery.is_low(20):
			self.leds.set_battery_status(Led.MODE_BLINK_4HZ)
		else:
			self.leds.set_battery_status(Led.MODE_ON)

	def _get_status(self):
		"""Get device status"""
		with self.state_lock:
			location = self.last_location
			sleeping = self.sleep_mode
			gps_valid = self.gps_available
			connected = self.connected
		status = 'GPS Tracker Status:\n'
		status += 'Battery: {}%{}\n'.format(self.battery.get_percentage(), ' (Charging)' if self.battery.is_charging else '')
		status += 'Voltage: {:.2f}V\n'.format(self.battery.voltage)
		status += 'Sleep: {}\n'.format('Yes' if sleeping else 'No')
		status += 'GPS: {}\n'.format('Valid' if gps_valid else 'Invalid')
		if location and location.get('valid'):
			status += 'Source: {}\n'.format(location.get('source', 'unknown'))
			status += 'Lat: {:.6f}\n'.format(location['latitude'])
			status += 'Lon: {:.6f}\n'.format(location['longitude'])
			status += 'Speed: {:.1f} km/h\n'.format(location.get('speed', 0))
			status += 'Sats: {}\n'.format(location.get('satellites', 0))
		status += 'Buffer: {} records\n'.format(self.data_buffer.size())
		status += 'Connected: {}\n'.format('Yes' if connected else 'No')
		gc.collect()
		status += 'Memory free: {} bytes'.format(gc.mem_free())
		return status

	def _poweroff(self):
		"""Power off device"""
		print('Powering off device...')
		try:
			self.config.save()
			self.cleanup()
			utime.sleep(1)
			Power.powerDown()
		except Exception as e:
			print('Poweroff error:', e)

	def _reset(self):
		"""Reset device"""
		print('Resetting device...')
		try:
			self.config.save()
			self.cleanup()
			utime.sleep(1)
			Power.powerRestart()
		except Exception as e:
			print('Reset error:', e)

	def cleanup(self):
		"""Cleanup resources"""
		print('Cleaning up...')
		self.running = False
		self.watchdog.stop()
		self.gps.disable()
		self.wifi_scanner.disable()
		self.leds.cleanup()
		if self.protocol:
			self.protocol.disconnect()
		print('Cleanup complete')


if __name__ == '__main__':
	try:
		print('=== GPS Tracker Starting ===')
		imei = modem.getDevImei()
		print('IMEI:', imei)
		tracker = GPSTracker()
		while True:
			utime.sleep(60)
			gc.collect()
			free_mem = gc.mem_free()
			total_mem = gc.mem_free() + gc.mem_alloc()
			mem_percent = (free_mem / total_mem) * 100
			print('Memory: {} bytes free ({:.1f}%)'.format(free_mem, mem_percent))
	except KeyboardInterrupt:
		print('Interrupted by user')
		tracker.cleanup()
	except Exception as e:
		print('Fatal error:', e)
		import sys
		sys.print_exception(e)
		try:
			tracker.cleanup()
		except:
			pass
