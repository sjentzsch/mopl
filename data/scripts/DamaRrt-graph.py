## in console: execfile('DamaRrt-graph.py')

from math import sqrt
from itertools import izip
from numpy.random import randint
from functools import partial
from graph_tool.all import *

def markEdges(g, v, e_prop):
	for e in v.out_edges():
		e_prop[e] = 1
		markEdges(g, e.target(), e_prop)

def getFirstIndex(g, v_prop, v_value):
	for v in g.vertices():
		if v_prop[v] == v_value:
			return g.vertex_index[v]

## Load the meta properties from the meta file
lines = [line.strip() for line in open('DamaRrt-graph-meta')]
metaTwoTreeVersion = int(lines[0])
metaFinished = 0
metaEnd0Index = 0
metaEnd1Index = 0
if len(lines) > 1:
	metaFinished = 1
	metaEnd0Index = int(lines[1])
	if metaTwoTreeVersion:
		metaEnd1Index = int(lines[2])
print 'metaTwoTreeVersion: ' + str(metaTwoTreeVersion)
print 'metaFinished: ' + str(metaFinished)
print 'metaEnd0Index: ' + str(metaEnd0Index)
print 'metaEnd1Index: ' + str(metaEnd1Index)

## Load the graphs and retrieve the existing properties

g = load_graph("DamaRrt-graph-0.xml")
g.set_directed(True)
num_vertices_g0 = g.num_vertices()
num_edges_g0 = g.num_edges()

v_index = g.vertex_properties["index"]
v_q = g.vertex_properties["q"]
v_radius = g.vertex_properties["radius"]

e_action = g.edge_properties["action"]

if metaTwoTreeVersion:
	g1 = load_graph("DamaRrt-graph-1.xml")
	g1.set_directed(True)
	num_vertices_g1 = g1.num_vertices()
	num_edges_g1 = g1.num_edges()

	v1_index = g1.vertex_properties["index"]
	v1_q = g1.vertex_properties["q"]
	v1_radius = g1.vertex_properties["radius"]

	e1_action = g1.edge_properties["action"]

## Add g1 to g to create the final connected-tree and create a new property storing the origin of the vertices (0, resp. 1)

v_tree = g.new_vertex_property("bool")

for v in g.vertices():
	v_tree[v] = 0

def copyTree(v1, v0):
	v_index[v0] = v1_index[v1] + num_vertices_g0
	v_q[v0] = v1_q[v1]
	v_radius[v0] = v1_radius[v1]
	v_tree[v0] = 1
	for e1 in v1.out_edges():
		v0e = g.add_vertex()
		e0 = g.add_edge(v0, v0e)
		e_action[e0] = e1_action[e1]
		copyTree(e1.target(), v0e)
if metaTwoTreeVersion:
	for v1 in g1.vertices():
		if v1.in_degree() == 0:
			v0 = g.add_vertex()
			copyTree(v1, v0)
num_vertices_g = g.num_vertices()
num_edges_g = g.num_edges()

g.vertex_properties["tree"] = v_tree

## Print some information

print 'Forward-Tree: ' + str(num_vertices_g0) + ' Vertices and ' + str(num_edges_g0) + ' Edges'
if metaTwoTreeVersion:
	print 'Backward-Tree: ' + str(num_vertices_g1) + ' Vertices and ' + str(num_edges_g1) + ' Edges'
	print '=> Compound Tree: ' + str(num_vertices_g) + ' Vertices and ' + str(num_edges_g) + ' Edges'

## Create new properties: v_q2 and v_pos

v_q2 = g.new_vertex_property("vector<float>")
v_pos = g.new_vertex_property("vector<float>")

for v in g.vertices():
	v_q2[v] = map(float, v_q[v].split(','))
	v_pos[v] = v_q2[v][0:2]
	v_pos[v][1] = -v_pos[v][1]

g.vertex_properties["q2"] = v_q2
g.vertex_properties["pos"] = v_pos

## Calculate some vertices: v_end0, v_start0, v_end1, v_start1

if metaFinished:
	v_end0 = g.vertex(getFirstIndex(g, v_index, metaEnd0Index))
	v = v_end0
	#print v_q2[v_end0]
	while v.in_degree() > 0:
		v = v.in_edges().next().source()
		#print v_q2[v]
	v_start0 = v

	if metaTwoTreeVersion:
		v_end1 = g.vertex(getFirstIndex(g, v_index, metaEnd1Index + num_vertices_g0))
		v = v_end1
		#print v_q2[v_end1]
		while v.in_degree() > 0:
			v = v.in_edges().next().source()
			#print v_q2[v]
		v_start1 = v

## Set filtering properties

e_special = g.new_edge_property("bool")
for e in g.edges():
	if (v_q2[e.source()][4] == -1.0) and (v_q2[e.source()][5] == 1.5) and (v_q2[e.target()][4] == -1.0) and (v_q2[e.target()][5] == 1.5):
		e_special[e] = 0
	else:
		e_special[e] = 1
g_special = GraphView(g, efilt=e_special)

# Color edges according to the action
e_color = g.new_edge_property("string")
for e in g.edges():
	if e_action[e] == "Transit":
		e_color[e] = "#000000"
	elif e_action[e].endswith("Object 1"):
		e_color[e] = "#ff0000"
	elif e_action[e].endswith("Object 2"):
		e_color[e] = "#00b500"
	elif e_action[e].endswith("Object 3"):
		e_color[e] = "#0000ff"
	else:
		e_color[e] = "#bdbf00"

# Mark start, connection point, goal, direction (arrows)
v_size = g.new_vertex_property("int")
v_shape = g.new_vertex_property("string")
v_fill_color = g.new_vertex_property("string")
for v in g.vertices():
	v_size[v] = 0
	v_shape[v] = "circle"
	v_fill_color[v] = "#000000"
if metaFinished:
	v_size[v_start0] = 15
	v_shape[v_start0] = "double_circle"
	if metaTwoTreeVersion:
		v_size[v_end0] = 20
		v_shape[v_end0] = "square"
		v_fill_color[v_end0] = "#d5801b"
		v_size[v_start1] = 15
		v_shape[v_start1] = "circle"
	else:
		v_size[v_end0] = 15
		v_shape[v_end0] = "circle"
e_mid_marker = g.new_edge_property("string")
for e in g.edges():
	e_mid_marker[e] = "none"
if metaFinished:
	e_mid_marker[v_end0.in_edges().next()] = "arrow"
	v = v_end0
	while v != v_start0:
		for e in v.in_edges():
			if e.source() == v_start0:
				e_mid_marker[e] = "arrow"
			v = e.source()

# Strengthen edges belonging to the path from start to goal (= last added vertex)
e_size = g.new_edge_property("double")
for e in g.edges():
	e_size[e] = 0.5
if metaFinished:
	v = v_end0
	while v.in_degree() > 0:
		for e in v.in_edges():
			e_size[e] = 5.0
			v = e.source()
	if metaTwoTreeVersion:
		v = v_end1
		while v.in_degree() > 0:
			for e in v.in_edges():
				e_size[e] = 5.0
				v = e.source()

e_action_transit = g.new_edge_property("bool")
for e in g.edges():
	if e_action[e] == "Transit":
		e_action_transit[e] = 1
	else:
		e_action_transit[e] = 0
g_f2 = GraphView(g, efilt=e_action_transit)
g_f2.set_edge_filter(e_action_transit, inverted=False)

e_tree = g.new_edge_property("bool")
for e in g.edges():
	if (v_tree[e.source()] == 0) and (v_tree[e.target()] == 0):
		e_tree[e] = 1
	else:
		e_tree[e] = 0
g_f3 = GraphView(g, efilt=e_tree)
g_f3.set_edge_filter(e_tree, inverted=False)

e_color_tree = g.new_edge_property("string")
for e in g.edges():
	if e_tree[e] == 0:
		e_color_tree[e] = "#ff0000"
	else:
		e_color_tree[e] = "#0000ff"

e_succ_of_v = g.new_edge_property("bool")
for e in g.edges():
	e_succ_of_v[e] = 0
markEdges(g, g.vertex(0), e_succ_of_v)	# mark here which vertex index
g_f4 = GraphView(g, efilt=e_succ_of_v)
g_f4.set_edge_filter(e_succ_of_v, inverted=False)

## Draw the graph

graph_draw(g, vertex_size=v_size, vertex_shape=v_shape, vertex_color="#000000", vertex_fill_color=v_fill_color, vertex_pen_width=1.7, pos=v_pos, edge_pen_width=e_size, edge_color=e_color, edge_marker_size=20, edge_start_marker="none", edge_mid_marker=e_mid_marker, edge_end_marker="none", output_size=(800, 800), output="graph-draw1.pdf") # increasing the output_size = zooming out = not good

graph_draw(g, vertex_size=v_size, vertex_shape=v_shape, vertex_color="#000000", vertex_fill_color=v_fill_color, vertex_pen_width=1.7, pos=v_pos, edge_pen_width=e_size, edge_color=e_color, edge_marker_size=20, edge_start_marker="none", edge_mid_marker=e_mid_marker, edge_end_marker="none", display_props=[v_tree, v_index, v_q], display_props_size=15, output_size=(800, 800), geometry=(800, 800))

if metaTwoTreeVersion:
	graph_draw(g, vertex_size=v_size, vertex_shape=v_shape, vertex_color="#000000", vertex_fill_color=v_fill_color, vertex_pen_width=1.7, pos=v_pos, edge_pen_width=e_size, edge_color=e_color_tree, edge_marker_size=20, edge_start_marker="none", edge_mid_marker=e_mid_marker, edge_end_marker="none", display_props=[v_tree, v_index, v_q], display_props_size=15, output_size=(800, 800), geometry=(800, 800))

	graph_draw(g_f3, vertex_size=v_size, vertex_shape=v_shape, vertex_color="#000000", vertex_fill_color=v_fill_color, vertex_pen_width=1.7, pos=v_pos, edge_pen_width=e_size, edge_color=e_color, edge_marker_size=20, edge_start_marker="none", edge_mid_marker=e_mid_marker, edge_end_marker="none", display_props=[v_tree, v_index, v_q], display_props_size=15, output_size=(800, 800), geometry=(800, 800))

	g_f3.set_edge_filter(e_tree, inverted=True)

	graph_draw(g_f3, vertex_size=v_size, vertex_shape=v_shape, vertex_color="#000000", vertex_fill_color=v_fill_color, vertex_pen_width=1.7, pos=v_pos, edge_pen_width=e_size, edge_color=e_color, edge_marker_size=20, edge_start_marker="none", edge_mid_marker=e_mid_marker, edge_end_marker="none", display_props=[v_tree, v_index, v_q], display_props_size=15, output_size=(800, 800), geometry=(800, 800))
