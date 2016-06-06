# third version of the knowledge graph using networkx 

# network toolkit
import networkx as nx
import os
# regular expressions 
import re
# drawing


#########################
# functions for file management

def load_semantic_network():
	# load several files into a single nx graph
	filePath = os.path.dirname(os.path.abspath(__file__))
	S = load_file_to_graph(filePath +"/ontologies/stuff_ontology.txt")
	A = load_file_to_graph(filePath + "/ontologies/attribute_ontology.txt")
	C = load_file_to_graph(filePath + "/ontologies/context_knowledge.txt")
	X = load_file_to_graph(filePath + "/ontologies/activity_ontology.txt")
	
	G = nx.compose(S,A)
	G = nx.compose(G,C)
	G = nx.compose(G,X)
	return G

def load_file_to_graph(file_name):
	# transform a file into a nx graph
	# params string file name
	# return nx graph
	G = nx.DiGraph()
	triads = read_matching_reg_exp('\([\?\w]+\s+\w+\s+\w+\)\s+', file_name)
	for each in triads:
		# print each
		G.add_node(each[0])
		G.add_node(each[2])
		G.add_edge(each[0], each[2], {'relation':each[1]})
	return G

def read_matching_reg_exp(reg_exp, file_name) :
	# read well formed nodes from lines of text
	# params regular expression of facts, string file name
	# return list of triads
	file = open(file_name, 'r')
	relations = []
	lines = file.readlines()
	for line in lines:
		if re.match( reg_exp, line):
			relation = re.split('[^a-zA-Z0-9\?_]+', line)[1:4]
			relations.append(relation)
	file.close()
	return relations

def transform_fact_list_to_graph(facts_list):
	# transform list of triads into nx graph
	# params list of triads
	# return nx graph
	G = nx.DiGraph()
	for each in facts_list:
		G.add_node(each[0])
		G.add_node(each[2])
		G.add_edge(each[0], each[2], {'relation':each[1]})
	return G

def add_edges_from_list(facts_list, file_name):
	# add list of triads into a file
	# params list of triads, string file name
	file = open(file_name, 'a')
	if isinstance(facts_list, list):
		for each_fact in facts_list:
			if len(each_fact) == 3:
				file.write("(" + each_fact[0] + "    " + each_fact[1] + "    " + each_fact[2] +") \n")
	file.close()
	return "understood"

def delete_edges_from_list(facts_list, file_name):
	# delete a list of triads in a file
	# params list of triads, string file name
	# return nothing
	for j in range(0, len(facts_list)):
		for i in range(0, 3):
			if facts_list[j][i] == "*":
				facts_list[j][i] = '\w+'
	file = open(file_name, 'r')
	lines = file.readlines()
	file.close()
	file = open(file_name, 'w')
	for each_line in lines:
		must_write = True
		for each_node in facts_list:
			expresion = "\(" + each_node[0] + "\s+" + each_node[1] + "\s+" + each_node[2] + "\)\s+" 
			if re.match(expresion, each_line):
				must_write = False
		if must_write:
			file.write(each_line)
	file.close()


#########################
# functions for GUI

def draw_graph(G):
	#nx.draw_circular(G)
	plt.figure()
	pos=nx.spring_layout(G) # positions for all nodes

	# nodes
	nx.draw_networkx_nodes(G,pos,node_size=2000, node_color="red")

	# edges
	nx.draw_networkx_edges(G,pos,width=1,alpha=0.7,edge_color='black')

	nx.draw_networkx_labels(G,pos,font_size=10,font_family='sans-serif')

	edges_dict = {}
	for nA, nbrs in G.adjacency_iter():
		for nB, att in nbrs.items():
			rel = att['relation']
			edges_dict[(nA,nB)]=rel

	nx.draw_networkx_edge_labels(G,pos, edges_dict)
	plt.show()



#########################
# accesing ontology

def all_subclasses(G, clss):
	# explore subclases given by relation is_kind_of
	# params nx graph, string node
	# return set of all inherited subclases
	if clss in G:
		fringe = [clss]
		subclases = []
		while fringe != []:
			current = fringe.pop(0)
			neighbors = G.predecessors(current)
			for each in neighbors:
				if G[each][current]['relation'] == 'is_kind_of':
					fringe.append(each)
					if each not in subclases:
						subclases.append(each)
		return subclases
	else:
		return []

def all_superclasses(G, clss):
	# explore superclases given by relation is_kind_of
	# params nx graph, string node
	# return set of all inherited superclasses
	if clss in G:
		fringe = [clss]
		superclasses = []
		while fringe != []:
			current = fringe.pop(0)
			neighbors = G.successors(current)
			for each in neighbors:
				if G[current][each]['relation'] == 'is_kind_of' or G[current][each]['relation'] == 'is_object_of':
					fringe.append(each)
					if each not in superclasses:
						superclasses.append(each)
		return superclasses
	else:
		return []

def all_objects(G, clss):
	# explore subclases given by relation is_object_of
	# params nx graph, string node
	# return set of all objects of all subclasses
	if is_object(G, clss):
		return [clss]
	if clss in G:
		fringe = [clss]
		objects = []
		while fringe != []:
			current = fringe.pop(0)
			neighbors = G.predecessors(current)
			for each in neighbors:
				if G[each][current]['relation'] == 'is_kind_of':
					fringe.append(each)
				elif G[each][current]['relation'] == 'is_object_of' and each not in objects:
					objects.append(each)
		return objects
	else:
		return []

def all_aka(G, objclss):
	akas = get_attribute_sym(G, objclss, 'aka')
	if 'aka' in akas:
		return akas['aka']
	else:
		return [objclss]
















def language_info():
	# look in the language_pos.txt file and load categories of
	# POS in a dictionary
	filePath = os.path.dirname(os.path.abspath(__file__))
	L = load_file_to_graph(filePath + '/ontologies/language_knowledge.txt')
	language_tags = {}
	for nA, nbrs in L.adjacency_iter():
		for nB, att in nbrs.items():
			rel = att['relation']
			if rel == 'tag':
				language_tags = merge_dictionaries(language_tags, {nB : [nA]})
	
	G = load_semantic_network()
	n = [all_aka(G, objclss) for objclss in all_subclasses(G, 'stuff')] + [all_aka(G, objclss) for objclss in all_objects(G, 'stuff')]
	nouns = []
	#print "nounS PREVIO------> ", n
	for each_aka in n:
		nouns += each_aka
	#print "nounS ------> ", nouns
	nouns = list_merge(nouns, all_subclasses(G, 'stuff') + all_objects(G, 'stuff'))

	language_tags.update({ 'noun': nouns })


	a = [all_aka(G, objclss) for objclss in all_subclasses(G, 'attribute')]
	attris = []
	#print "nounS PREVIO------> ", n
	for each_aka in a:
		attris += each_aka
	attris = list_merge(attris, all_subclasses(G, 'attribute'))


	ad = [all_aka(G, objclss) for objclss in all_objects(G, 'attribute')]
	ads = []
	#print "nounS PREVIO------> ", n
	for each_aka in ad:
		ads += each_aka
	ads = list_merge(ads, all_objects(G, 'attribute'))



	language_tags.update({'adj' : ads })
	language_tags.update({'att' : attris})
	return language_tags



























def is_type(G, semantic_type, objclss):
	sub_types = all_subclasses(G, semantic_type) + all_objects(G, semantic_type)
	akas = all_aka(G, objclss) + [objclss]
	print 'sub_types: ', sub_types
	print 'akas', akas
	return not len(list_diff(akas, sub_types)) == len(akas) 

def compound_words(G):
	n = [all_aka(G, objclss) for objclss in all_subclasses(G, 'stuff')] + [all_aka(G, objclss) for objclss in all_objects(G, 'stuff')]
	nouns = []
	#print "nounS PREVIO------> ", n
	for each_aka in n:
		nouns += each_aka

	a = [all_aka(G, objclss) for objclss in all_subclasses(G, 'attribute')] + [all_aka(G, objclss) for objclss in all_objects(G, 'attribute')]
	attris = []
	#print "nounS PREVIO------> ", n
	for each_aka in a:
		attris += each_aka

	words = attris + nouns
	compound_words = []
	for each in words:
		if '_' in each:
			compound_words.append(each)
	return compound_words


def find_original_aka(G, aka_word):
	if aka_word not in G:
		return aka_word

	n = [all_aka(G, objclss) for objclss in all_subclasses(G, 'stuff')] + [all_aka(G, objclss) for objclss in all_objects(G, 'stuff')]
	nouns = []
	#print "nounS PREVIO------> ", n
	for each_aka in n:
		nouns += each_aka

	nouns = list_merge(nouns, all_subclasses(G, 'stuff') + all_objects(G, 'stuff'))

	a = [all_aka(G, objclss) for objclss in all_subclasses(G, 'attribute')] + [all_aka(G, objclss) for objclss in all_objects(G, 'attribute')]
	attris = []
	#print "nounS PREVIO------> ", n
	for each_aka in a:
		attris += each_aka

	attris = list_merge(attris, all_subclasses(G, 'attribute') + all_objects(G, 'attribute'))

	words = attris + nouns


	for each_word in words:
		akas = all_aka(G, each_word)
		if akas != [] and each_word not in akas and aka_word in akas:
			return each_word


	return aka_word

#########################
# accesing values of attributes

def get_attribute(G, objclss, attribute):
	# look for all alias of object or class and create a dict for 
	# each alias, then combine them to a final dict
	akas = all_aka(G, objclss)
	#print 'all akas: ', akas
	values_dictionary = {}
	for each in akas:
		values_dictionary = merge_dictionaries(values_dictionary, get_attribute_sym(G, each, attribute))
	return values_dictionary

def get_attribute_sym(G, objclss, attribute):
	# look for the value of an attribute considering if it is transitive
	# or symmetric
	# return a list of tuples of (attribute, value)
	if objclss in G and attribute in G:
		locally_defined = is_locally_defined(G, attribute)
		fringe = [objclss]
		explored = [objclss]	
		property_value = []
		sub_attributes = [attribute] + all_subclasses(G, attribute) + all_objects(G, attribute)
		collected_relations = {}
		while fringe != []:
			current = fringe.pop(0)
			neighbors = G.successors(current)
			neighbors_sym = G.predecessors(current)
			complete_neighborhood = neighbors + neighbors_sym
			#print 'normal: ', G.successors(current)
			#print 'symmetric: ', G.predecessors(current)
			for each in complete_neighborhood:
				# case forward direction of property
				if each in neighbors:
					if G[current][each]['relation'] in sub_attributes:
						transitive = is_transitive_attribute(G, G[current][each]['relation'])
						unique_value = has_unique_value_attribute(G, G[current][each]['relation'])
						if transitive and G[current][each]['relation'] not in collected_relations:
							collected_relations.update(get_dict_attribute_path(G, G[current][each]['relation'], each))
						elif G[current][each]['relation'] not in collected_relations:
							collected_relations.update({G[current][each]['relation'] : [each]})
						# if should append new value to list
						if transitive and G[current][each]['relation'] in collected_relations and not unique_value:
							collected_relations = merge_dictionaries(collected_relations, get_dict_attribute_path(G, G[current][each]['relation'], each))
						elif G[current][each]['relation'] in collected_relations and not unique_value:
							collected_relations = merge_dictionaries(collected_relations, {G[current][each]['relation'] : [each]})

					if G[current][each]['relation'] == 'is_kind_of' or G[current][each]['relation'] == 'is_object_of':
						if each not in explored and not locally_defined:
							fringe.append(each)
				if each in neighbors_sym:
					if G[each][current]['relation'] in sub_attributes:
						symmetric = is_symmetric_attribute(G, G[each][current]['relation'])	
						transitive = is_transitive_attribute(G, G[each][current]['relation'])
						unique_value = has_unique_value_attribute(G, G[each][current]['relation'])	
						
						if symmetric and transitive and G[each][current]['relation'] not in collected_relations:
							collected_relations.update(get_dict_attribute_path(G, G[each][current]['relation'], each))	
						elif symmetric and not transitive and G[each][current]['relation'] not in collected_relations:
							collected_relations.update({G[each][current]['relation'] : [each]})
						# if sholud append values
						if symmetric and transitive and G[each][current]['relation'] in collected_relations and not unique_value:
							collected_relations = merge_dictionaries(collected_relations, get_dict_attribute_path(G, G[each][current]['relation'], each))
						elif symmetric and not transitive and G[each][current]['relation'] in collected_relations and not unique_value:
							#collected_relations.update({G[each][current]['relation'] : [each]})
							collected_relations = merge_dictionaries(collected_relations, {G[each][current]['relation'] : [each]})
				
				explored.append(each)

		return collected_relations
	else:
		return {}

def get_dict_attribute_path(G, transitive_attribute, value):
	#return dictionary
	collected_transitive = {transitive_attribute : [value]}
	fringe = [value]
	explored = [value]
	symmetric = is_symmetric_attribute(G, transitive_attribute)
	while fringe != []:
		current = fringe.pop(0)
		neighbors = G.successors(current)
		for each in neighbors:
			if G[current][each]['relation'] == transitive_attribute:
				if each not in explored:
					fringe.append(each)
					collected_transitive[transitive_attribute].append(each)
			explored.append(each)
	return collected_transitive

def verify_satisfability_of_objclss(G, objclss, values_list):
	# get all the attribute of objclss
	attribute_dict = get_attribute(G, objclss, 'attribute')
	remaining_values = values_list[:]
	# for each value to verify check if it is contain in the dictionary
	for each_value in values_list:
		for each_attribute in attribute_dict:
			if each_value in attribute_dict[each_attribute]:
				remaining_values.remove(each_value)
	# check if all values were satisfied
	if remaining_values == []:
		return True
	else:
		return False

def semantic_type(G, cls):
	superclasses = all_superclasses(G,cls)
	if "person" in superclasses:
		return "person"
	elif "attribute" in superclasses:
		return "attribute"
	elif "place" in superclasses:
		return "place"
	elif "item" in superclasses:
		return "item"
	elif "passive_activity" in superclasses or "command_activity" in superclasses:
		return "action"
	
	else:
		return "none"		

#########################
# accesing properties of attributes

def is_transitive_attribute(G, attribute):
	# verify if an attribute is declared transitive
	# return bool
	if attribute in G:
		neighbors = G.successors(attribute)
		if 'transitive' in neighbors:
			if G[attribute]['transitive']['relation'] == 'algebraic_property':
				return True
			else:
				return False
		else:
			return False
	else:
		return False

def has_unique_value_attribute(G, attribute):
	# verify if an attribute is declared to have only one value per object
	# return bool
	if attribute in G:
		neighbors = G.successors(attribute)
		if 'has_unique_value' in neighbors:
			if G[attribute]['has_unique_value']['relation'] == 'algebraic_property':
				return True
			else:
				return False
		else:
			return False
	else:
		return False

def is_symmetric_attribute(G, attribute):
	# verify if an attribute is declared transitive
	# return bool
	if attribute in G:
		neighbors = G.successors(attribute)
		if 'symmetric' in neighbors:
			if G[attribute]['symmetric']['relation'] == 'algebraic_property':
				return True
			else:
				return False
		else:
			return False
	else:
		return False

def is_locally_defined(G, attribute):
	# verify if an attribute is declared transitive
	# return bool
	if attribute in G:
		neighbors = G.successors(attribute)
		if 'locally_defined' in neighbors:
			if G[attribute]['locally_defined']['relation'] == 'algebraic_property':
				return True
			else:
				return False
		else:
			return False
	else:
		return False



def is_object(G, obj):
	# verify if an attribute is declared transitive
	# return bool
	if obj in G:
		neighbors = G.successors(obj)
		#print neighbors
		for each_nei in neighbors:
			#print "testing: ", obj, " and ", each_nei, "relation: ", G[obj][each_nei]['relation']
			if G[obj][each_nei]['relation'] == 'is_object_of':
				return True
			
		else:
			return False
	else:
		return False

#########################
# utils

def merge_dictionaries(dict_a, dict_b):
	merged = dict_a.copy()
	for each in dict_b:
		if each in merged:
			merged[each] = list_merge(merged[each], dict_b[each])
		else:
			merged.update({each: dict_b[each]})
	return merged 

def list_merge(A, B):
	merge = A[:]
	for b in B:
		if b not in merge:
			merge.append(b)  
	return merge

def list_diff(A,B):
		diff = []
		for a in A:
			if a not in B:
				diff.append(a)  
		return diff


def intersection(a,b):
	if isinstance(a, str):
		a = [a]
	a = set(a)
	return [bb for bb in b if bb in a]


def get_objects_that_match(G, objclss, values_list):
	return [o for o in all_objects(G, objclss) if verify_satisfability_of_objclss(G, o, values_list)]


def get_objects_that_match2(G, objclss, attribute, values_list):
	o_final = []
	o_list = all_objects(G, objclss)
	for o in o_list:
		dic = get_attribute(G, o, attribute)
		if not dic == {} and len(intersection(dic[attribute], values_list)) > 0:
			#print "dictionary: ", dic, "       values_list: ", values_list, "    inter:", intersection(dic[attribute], values_list)
			o_final.append(o)
	
	return o_final




G = load_semantic_network()
#print 'get all attributes and values: ', get_attribute(G, 'fanta_1', 'temperature')


#print "what is cold? ", verify_satisfability_of_objclss(G, "sam", ["kitchen", "in"])

#print "lolcat", get_objects_that_match(G, "stuff", ["cold"])


#print "lolcat", get_objects_that_match2(G, "man", "in", ["livingroom_1"])

#print "lolcat", is_object(G, "tableLoc0")

#print "objs: ", all_objects(G,"sam")

#print "test aka: ", all_aka(G, "tableLoc0")
#print "test original aka", find_original_aka(G, "brick")

#print "test " + all_superclasses(G,"red")[0]
