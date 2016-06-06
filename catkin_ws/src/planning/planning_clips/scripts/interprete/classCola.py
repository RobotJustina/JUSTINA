class classCola:
	
	def __init__(self):
		self.items = []
	
	def pushC(self, x):
		self.items.append(x)
		#print (self.items)
	
	def popC(self):
		try:
			print (self.items[0])
			return self.items.pop(0)
		except:
			raise ValueError("La cola esta vacia")

	def es_vacia(self):
		return self.items == []

	def lenC(self):
		return len(self.items)
	
	def empty(self):
		self.items = []

	def popPile(self):
		try:
			return self.items.pop()
		except:
			raise ValueError("La cola esta vacia")
