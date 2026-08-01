try:
    import ujson
except ImportError:
    import json as ujson

try:
    import uos
except ImportError:
    import os as uos

from misc import Power

CONFIG_FILE = '/usr/tracker_config.json'
DEFAULT_CONFIG = {
    'apn': {'name': 'internet', 'user': '', 'password': ''},
    'server': {'protocol': 'GT06', 'host': '', 'port': 0, 'path': '/api/location'},
    'wifi_server': None,
    'wifi_location_enabled': False,
    'update_interval': 10,
    'sleep_timeout': 3600,
    'buffer_enabled': True,
    'sms_numbers': [],
    'battery_low_threshold': 20,
    'low_power_gps': True,
    'battery_calibration': False,
    'accelerometer_enabled': False
}

class Config:
    """Configuration manager for GPS Tracker"""

    def __init__(self):
        self.config = self._load()
        self.save()
        print('Configuration loaded')

    def _load(self):
        """Load configuration from file"""
        try:
            with open(CONFIG_FILE, 'r') as f:
                config = ujson.load(f)
            for key, value in DEFAULT_CONFIG.items():
                if key not in config:
                    config[key] = value
            config.pop('imei', None)
            if isinstance(config.get('wifi_server'), str):
                config['wifi_server'] = self._parse_wifi_server(config['wifi_server'])
            return config
        except Exception as e:
            print('Config load error:', e)
        return DEFAULT_CONFIG.copy()

    def _parse_wifi_server(self, value):
        """Parse legacy wifi_server string ('http://host:port') into dict"""
        ws = {'host': '', 'port': 5055, 'path': '/api/locate'}
        try:
            value = value.strip()
            if '://' in value:
                value = value.split('://', 1)[1]
            if '/' in value:
                host_port, path = value.split('/', 1)
                ws['path'] = '/' + path
            else:
                host_port = value
            if ':' in host_port:
                host, port = host_port.split(':', 1)
                ws['host'] = host
                ws['port'] = int(port)
            else:
                ws['host'] = host_port
        except Exception as e:
            print('WiFi server parse error:', e)
        return ws

    def save(self):
        """Save configuration to file"""
        try:
            with open(CONFIG_FILE, 'w') as f:
                ujson.dump(self.config, f)
            print('Configuration saved')
            return True
        except Exception as e:
            print('Config save error:', e)
        return False

    def get(self, key, default=None):
        """Get configuration value"""
        return self.config.get(key, default)

    def update(self, **kwargs):
        """Update configuration"""
        for key, value in kwargs.items():
            self.config[key] = value
        return self.save()

    def reset(self):
        """Reset to default configuration"""
        self.config = DEFAULT_CONFIG.copy()
        return self.save()