
# network toolkit
import networkx as nx
import kb_services
import re
import networkx.drawing

# por diseno el primer valor de la matriz es el simbolo de inicio S
#grammar for noun frases
unknown_counter = 0
unknown_list = []
grammar_np_simple = [
	['NP',			'prep_loc',				'NP'],
	['NP',			'PREPDET',				'NP'],
	['PREPDET',		'prep_loc',				'DETNOUN'],
	['DETNOUN',		'det',					'adj'],
	['NP',			'existencial',			'noun'],
	['NP',			'universal',			'noun'],
	['NP',			'pro',					'noun'],
	['NP',			'number',				'noun'],
	['NP',			'det',					'noun'],
	['NP',			'det',					'att'],

	['NP',			'adj',					'noun'],
	['NP',			'ADJS',					'noun'],

	['NP',			'det',					'NP2'],
	['NP',			'det',					'NPnoun'],
	['NPnoun',		'noun',					'noun'],


	['NP',			'existencial',			'noun_q'],
	['NP',			'universal',			'noun_q'],
	['NP',			'number',				'noun_q'],
	['NP',			'det',					'noun_q'],
	['NP',			'pro',					'noun_q'],
	
	['noun_q',		'adj',					'noun'],
	['noun_q',		'ADJS',					'noun'],
	
	['ADJS',		'adj',					'and'],
	['ADJS',		'adj',					'adj'],
	['ADJS',		'ADJS',					'adj'],
	['ADJS',		'adj',					'ADJS'],
	
	['noun_q',		'adj',					'NP2'],
	['noun_q',		'ADJS',					'NP2'],

	['NP2',			'noun',					'NPad'],
	['NPad',		'adj',					'NP'],

	['NP',			'noun',					'NP1'],
	['NP1',			'prep_loc',				'noun'],
	['NP',			'existencial',			'NP'],
	['NP',			'universal',			'NP'],
	['NP',			'number',				'NP'],
	['NP',			'det',					'NP'],
	['ad_np',		'adj',					'NP'],

	# NOUN_PHRASE_1 that_is nearest to NOUN_PHRASE_2
	['NP',			'NP',					'comple'],
	['comple',		'rel_pro',				'comple2'],
	['comple',		'rel_pro',				'NP'],
	['comple2',		'det',					'noun'],
	['comple2',		'adj',					'comple3'],
	['comple2',		'att',					'comple3'],
	['comple2',		'att',					'NP'],
	['comple3',		'prep',					'NP'],
	['comple3',		'prep',					'noun'],
	
	
	
	


	['NP',			'idf_pro',				'TOVERB'],
	['TOVERB',		'particule',			'vrb']]


grammar_np_complex = [
	['NP'			,'existencial'			,'noun'],
	['NP'			,'existencial'			,'NP'],
	['NP'			,'universal'			,'noun'],
	['NP'			,'universal'			,'NP'],
	['NP'			,'det'					,'noun'],
	['NP'			,'det'					,'NP'],
	['NP'			,'pro'					,'adj'],
	['NP'			,'pos_pro'				,'NP'],
	['NP'			,'pos_pro'				,'noun'],
	['NP'			,'adj'					,'noun'],
	['NP'			,'adj'					,'NP'],
	['NP'			,'noun'					,'noun'],
	['NP'			,'pro'					,'A1'],
	['A1'			,'particule'			,'vrb']]


grammar_pp = [
	['PP'			,'prep_loc'				,'noun'],
	['PP'			,'prep'					,'noun'],
	['PP'			,'prep_time'			,'noun'],
	['PP'			,'prep_time'			,'time'],
	['PP'			,'prep_loc'				,'NP'],
	['PP'			,'prep_time'			,'NP'],
	['NP',			'existencial',			'noun'],
	['NP',			'universal',			'noun'],
	['NP',			'number',				'noun'],
	['NP',			'det',					'noun'],
	['NP',			'existencial',			'NP'],
	['NP',			'universal',			'NP'],
	['NP',			'number',				'NP'],
	['NP',			'det',					'NP'],
	['NP',			'adj',					'noun'],
	['NP',			'ADJS',					'noun'],
	['ADJS',		'adj',					'adj'],
	['ADJS',		'ADJS',					'adj'],
	['NP',			'idf_pro',				'TOVERB'],
	['TOVERB',		'particule',			'vrb']	
]

# POS tagging rules
# 1) pos tag n-grams that have preference
pos_ngrams = [
	['particule', 'vrb'], 
	['prep_loc', 'det'], 
	['prep', 'det'], 
	['prep', 'noun'], 
	['prep', 'existencial'], 
	['prep_loc', 'pro'], 
	['prep_loc', 'pos_pro'], 
	['prep_loc', 'universal'],
	['prep_loc', 'existencial'],
	['existencial', 'noun', 'rel_pro'],
	['universal', 'noun', 'rel_pro'],
	['det', 'noun', 'rel_pro'],
	['existencial', 'det', 'noun'],
	['universal', 'det', 'noun'],
	['existencial', 'noun'],
	['universal', 'noun'],
	['unknown', 'noun'],
	['number', 'noun'],
	['noun', 'vrb'],
	['det', 'noun'],	
	['number', 'adj'],
	['universal', 'adj', 'noun'],
	['existencial', 'adj', 'noun'],
	['number', 'noun', 'to', 'det'],
	['adj', 'noun'],
	['idf_pro', 'particule', 'vrb']]
# 2) pos tag bigrams which second element is a noun with semantic restrictions
pos_bigrams_noun_semantics = [
	['prep', 'place'],
	['prep', 'relation'], 
	['prep', 'person']]



#########################
# functions for mapping words into ontology lexems
def ontology_words_mapping(sentence):
	# simple preprocessing of utterance substitution of key words, phrases and compound words
	G = kb_services.load_semantic_network()
	language_info = kb_services.language_info()
	sentence = sentence.lower()
	# substitutes pronoun me for speaker (later should use contextual info of robot, who is talking?)
	#sentence = re.sub(' me ', ' speaker ', sentence)
	sentence = re.sub('\?', ' ?', sentence)
	sentence = re.sub('pick up ', 'take ', sentence)

	sentence = re.sub('  ', ' ', sentence)
	sentence = re.sub('\.', '', sentence)
	sentence = re.sub(', ', ' ', sentence)

	sentence = re.sub(' $', '', sentence)


	sentence = re.sub(' one ', ' 1 ', sentence)
	sentence = re.sub(' two ', ' 2 ', sentence)
	sentence = re.sub(' three ', ' 3 ', sentence)
	sentence = re.sub(' four ', ' 4 ', sentence)
	sentence = re.sub(' five ', ' 5 ', sentence)

	sentence = re.sub(' other ', ' ', sentence)

	sentence = re.sub(' person ', ' man ', sentence)

	sentence = re.sub(' which is ', ' that_is ', sentence)

	sentence = re.sub(' exactly down', ' down', sentence)

	sentence = re.sub('search for ', 'search_for ', sentence)

	sentence = re.sub('look for', 'look_for', sentence)

	
	sentence = re.sub('(could|can|would) you (please )?(robot )?', '', sentence)
	sentence = re.sub('please (robot )?', '', sentence)
	
	# declaration of classes and objects
	sentence = re.sub('((is)|(are)) ((an object of)|(an instance of)|(an adjetive of))( a)? ', 'is is_object_of ', sentence)
	sentence = re.sub('((is)|(are)) ((a kind of)|(a sort of)) ', 'is is_kind_of ', sentence)
	# simple from of verbs
	sentence = re.sub(' ((is)|(are)) ', ' is ', sentence)
	# transform plural into singular
	#for each in language_info['noun']:
	#	plural = derive_plural(each)
	#	sentence = re.sub(plural, each, sentence)
	# unite compound words
	compound_words = kb_services.compound_words(G)
	#print "compound nons :  ", compound_words
	for each in compound_words:
		test = re.sub('_', ' ', each)
		sentence = re.sub(test, each, sentence)

	#print "de aqui debo sustituir aka: ", sentence
	words = sentence.split(' ')
	new_sentence = []
	for each in words:
		new_sentence.append(kb_services.find_original_aka(G, each))
		#print "palabra: ",each, " se sustituye por:  ", kb_services.find_original_aka(G, each)

	new_sentence = " ".join(new_sentence)
	return new_sentence

def derive_plural(word):
	plural = word + 's'
	return plural

#########################
# functions for POS tagging 
def pos_tagger(G, sentence):
	# loads dictionary with language information given in ontologies
	language_info = kb_services.language_info()
	# brake sentence into words
	words = sentence.split(' ')
	tags = []
	# for each word look for all pos tags it has
	for w in words:
		tags.append(look_tags(language_info, w))
	
	ranked_disambiguated_tags = disambiguate_pos(G, words, tags)
	print "pos tags: ", ranked_disambiguated_tags
	return words, ranked_disambiguated_tags
	
def look_tags(voc, word):
	# recibes a dictionary of list of categories with the formar
	# {'pos_tag_1': [word_j, word_n, ..], 'pos_tag_2': [word_m, word_k, ..]}
	if re.match('[0-9]+', word):
		return ['number']
	result = []
	for each in voc:
		if word in voc[each]:
			result.append(each)
	if result != []:
		return result
	else:
		
		if word not in unknown_list:
			unknown_list.append(word)
		return ['unknown']


def disambiguate_pos(G, words, tags):	
	# create all combinations posible, then rank them
	all_combinations = [[]]
	for i in range(len(tags)):
		if len(tags[i]) == 1:
			for j in range(len(all_combinations)):
				all_combinations[j] = all_combinations[j] + tags[i]
		else:
			length = len(all_combinations)
			all_combinations = all_combinations * len(tags[i])
			for n in range(len(tags[i])):
				for j in range(length):
					all_combinations[n*length+j] = all_combinations[n*length+j] + [tags[i][n]]
	#print 'all combinations::::: ', all_combinations
	ranked_combinations = rank_pos_combinations(G, words, all_combinations)

	return ranked_combinations

def rank_pos_combinations(G, words, combinations):
	ranks = [0] * len(combinations)
	# iterate on possible combinations of pos
	for i in range(len(combinations)):
		# iterate on diferent starting pos[j]
		for j in range(len(combinations[0])):
			#iterate on different lengths of chunk
			for k in range(j,len(combinations[0])+1):
				# look if chunk of combination i is a n-gramm
				for each in pos_ngrams:
					#print "testing: ",  combinations[i][j:k], " contra ", each 
					if combinations[i][j:k] == each:
						#print "hit disambiguate rule"
						ranks[i] = ranks[i] + 1 
						
	#reranking
	#print "pos combination ranks: ", ranks
	reranked_combinations = []
	for i in range(len(combinations)):
		max_index = ranks.index(max(ranks))
		reranked_combinations =  reranked_combinations + [combinations[max_index]] 
		combinations.pop(max_index)
		ranks.pop(max_index)
	return reranked_combinations

def check_sublist(list_all, sublist):
	for i in range(len(list_all)):
		if list_all[i] == sublist[0]:
			if (i + len(sublist)) <= len(list_all):
				checked = True
				for j in range(len(sublist)):
					checked = checked and (list_all[i+j] == sublist[j])
				if checked:
					return True
			else:
				return False
	return False


def all_combinations(tags):
	all_combinations = [[]]
	for i in range(len(tags)):
		if len(tags[i]) == 1:
			for j in range(len(all_combinations)):
				all_combinations[j] = all_combinations[j] + tags[i]
		else:
			length = len(all_combinations)
			all_combinations = all_combinations * len(tags[i])
			for n in range(len(tags[i])):
				for j in range(length):
					all_combinations[n*length+j] = all_combinations[n*length+j] + [tags[i][n]]
	
	return all_combinations

#########################
# functions for syntactic parsing
def parser_cyk(grammar, tags):
	if len(tags) == 0:
		return False
	length = len(tags)
	parsing_matrix = [['null' for x in xrange(length)] for x in xrange(length)]
	#initializing matrix for CYK groups of 1
	for i in range(length):
		parsing_matrix[i][i] = tags[i]
	iterator = 1
	#CYK for groups of 2 and up
	#number of tiers in the tree
	for tier in range(1, length): # de 1 a numbero de palabras
		row = length - tier 
		column = length 
		for num_entry in range(length - tier, 0, -1): #de numero de palabras -1 menos el nivel hasta 1
			row = row - 1
			column = column - 1
			for num_cross_matches in range(1, tier + 1): #numero de casos a considerar por entry
				row_check1 = row  
				column_check1 = column - (tier - num_cross_matches + 1)
				column_check2 = column  
				row_check2 = row + num_cross_matches 
				temp = look_for_derivation(grammar, [ parsing_matrix[row_check1][column_check1],parsing_matrix[row_check2][column_check2]  ] )
				if temp != 'null':
					parsing_matrix[row][column] = temp
				#sentence = raw_input('Enter key to continue  ')
	#print "parsin matrix: ", parsing_matrix
	if parsing_matrix[0][length-1] == "null":
		return False
	elif parsing_matrix[0][length-1] == grammar[0][0]:
		return True
	else:
		return False

def look_for_derivation(grammar, derivation):
	#print "look for derivation: ", derivation
	for rule in grammar:
		if rule[1:3] == derivation:
			return rule[0]
	return 'null'

def constituent_chunker(grammar, list_words, pos_tags):
	lenght = len(list_words)
	words = list_words[:]
	pos = []
	np_list = []
	np_counter = 0
	current_start = 0
	best_chunk = [0,0]
	while current_start	< lenght:
		current_end = current_start + 1
		while current_end < lenght+1:
			test_tags = pos_tags[current_start:current_end]
			test_words = list_words[current_start:current_end]
			#print "trying parsing: ", test_tags, test_words
			check_np = parser_cyk(grammar, test_tags)
			if check_np:
				best_chunk = [current_start, current_end]
			current_end = current_end + 1
		if best_chunk != [0,0]:	
			test_tags = pos_tags[best_chunk[0]:best_chunk[1]]
			test_words = list_words[best_chunk[0]:best_chunk[1]]
			np_counter = np_counter + 1
			#print "Positive match: ", test_words
			for i in range(best_chunk[0], best_chunk[1]):
				words[i] = "NOUN_PHRASE_" + str(np_counter)
			np_list = np_list + [[test_words, test_tags]]
			current_start = best_chunk[1] -1
			pos.append("noun")
			best_chunk = [0,0]
		else:
			pos.append(pos_tags[current_start])
		current_start = current_start + 1
	no_duplicates_words = []
	for each in words:
		if each not in no_duplicates_words:
			no_duplicates_words = no_duplicates_words + [each]
	return [pos, no_duplicates_words, np_list]


def pp_chunker(grammar, list_words, pos_tags, np_list):
	lenght = len(list_words)
	words = list_words[:]
	pos = []
	pp_list = []
	np_counter = 0
	current_start = 0
	best_chunk = [0,0]
	while current_start	< lenght:
		current_end = current_start + 1
		while current_end < lenght+1:
			test_tags = pos_tags[current_start:current_end]
			test_words = list_words[current_start:current_end]
			#print "trying parsing: ", test_tags, test_words
			check_np = parser_cyk(grammar, test_tags)
			if check_np:
				best_chunk = [current_start, current_end]
			current_end = current_end + 1
		if best_chunk != [0,0]:	
			test_tags = pos_tags[best_chunk[0]:best_chunk[1]]
			test_words = list_words[best_chunk[0]:best_chunk[1]]
			np_counter = np_counter + 1
			#print "Positive match: ", test_words
			for i in range(best_chunk[0], best_chunk[1]):
				words[i] = "PREP_PHRASE_" + str(np_counter)
			pp_list = pp_list + [[test_words, test_tags]]
			current_start = best_chunk[1] -1
			pos.append("prep_phrase")
			best_chunk = [0,0]
		else:
			pos.append(pos_tags[current_start])
		current_start = current_start + 1
	no_duplicates_words = []
	for each in words:
		if each not in no_duplicates_words:
			no_duplicates_words = no_duplicates_words + [each]
	return [pos, no_duplicates_words, np_list, pp_list]








################################
# testing individual features
def test_ontology_word_mapping():
	sentence = 'where are all chairs functional status good'
	print ontology_words_mapping(sentence)

def test_pos():
	# graph of the knowledge base 
	G = kb_services.load_semantic_network()
	#print G.nodes()
	words, ranked_tags = pos_tagger(G, "something to eat")
	print  "words: ", words, "  tags: ", ranked_tags

def test_disambiguity():
	# graph of the knowledge base
	G = kb.load_semantic_network() 
	all_c = disambiguate_pos(G, [],[['uno'], ['a','b'], ['dos'], ['c', 'd'], ['tres']])
	print  "all combinations: ", all_c


def test_cyk():
	G = kb_services.load_semantic_network()
	#print G.nodes()
	words, ranked_tags = pos_tagger(G, "the kitchen")
	print  "words: ", words, "  tags: ", ranked_tags

	print "NP? ", parser_cyk(grammar_np_simple, ranked_tags[0])

def test_chunker():
	G = kb_services.load_semantic_network()
	#print G.nodes()
	sentence = "find the door on the right"
	compound_words = kb_services.compound_words(G)
	#print "compound nons :  ", compound_words
	for each in compound_words:
		test = re.sub('_', ' ', each)
		sentence = re.sub(test, each, sentence)

	words, ranked_tags = pos_tagger(G, sentence)	


	print words
	print ranked_tags[0]

	pp_interpretation = constituent_chunker(grammar_np_simple, words, ranked_tags[0])

	print "chunked words: ", pp_interpretation[1]

	print "noun phrases: ", pp_interpretation[2]

#test_ontology_word_mapping()
#test_pos()
#test_disambiguity()
#test_cyk()
#test_chunker()