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

        def insertElement(self, x, index):
                plan_inicio = self.items[:index-1]
                plan_final = self.items[index-1:]
                index_t = index + 1
                
                new_task = x;
                new_task[3] = "step " + str(index)
                
                plan_inicio.append(new_task)
                #plan_inicio.extend(plan_final)
                #print(plan_inicio)

                for i in plan_final:
                    task = i
                    task[3] = "step " + str(index_t)
                    index_t = index_t + 1
                    plan_inicio.append(task)
                #    if len(task)>1:
                #print(plan_inicio)
                self.items = plan_inicio
                print(self.items)

        def showQueue(self):
            print(self.items)


