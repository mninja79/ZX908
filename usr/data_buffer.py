import gc


class DataBuffer:
	"""Data buffer for offline storage"""

	def __init__(self, max_memory_percent=10, gc_interval=10):
		self.buffer = []
		self.max_memory_percent = max_memory_percent
		self.gc_interval = gc_interval
		self.gc_counter = 0

	def add(self, data):
		"""Add data to buffer"""
		if self._check_memory():
			self.buffer.append(data)
			return True
		return False

	def get_points(self):
		for i in range(len(self.buffer)):
			yield self.buffer[i]

	def clear(self):
		"""Clear buffer"""
		self.buffer.clear()
		gc.collect()

	def remove(self, count):
		"""Remove first count records"""
		self.buffer = self.buffer[count:]
		gc.collect()

	def size(self):
		"""Get buffer size"""
		return len(self.buffer)

	def _check_memory(self):
		"""Check available memory"""
		self.gc_counter += 1
		if self.gc_counter >= self.gc_interval:
			gc.collect()
			self.gc_counter = 0
		free = gc.mem_free()
		total = gc.mem_free() + gc.mem_alloc()
		free_percent = (free / total) * 100
		return free_percent >= self.max_memory_percent
