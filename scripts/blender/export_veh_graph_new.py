

"""
TODO:
    revisar margin, ahora no esta dividido por la longitud del vehiculo
"""

## \namespace export_veh_graph
## Exportador para Blender del grafo de navegación de vehículos.
## 
## **** PONER AQUI COMO INSTALAR EL PLUGIN EN BLENDER ****
## 
## \warning  El valor de las constantes SIGN_* y ROUTE_* debe ser igual a los de nav::veh::sign.


import bpy
import math
import struct


## Tamaño de las celdas. (metros)
CELL_SIZE = 32.0




## \cond PRIVATE

GROUP_NONE  = 0
GROUP_SPAWN = 1
GROUP_CROSS = 2
GROUP_LEFT  = 3
GROUP_RIGHT = 4
GROUP_YIELD = 5
GROUP_STOP  = 6
GROUP_SPEED     = 10000
GROUP_SEMAPHORE = 20000

SIGN_NONE       = 0
SIGN_SPAWN      = 1
SIGN_YIELD      = 2
SIGN_STOP       = 3
SIGN_SEMAPHORE  = 4
SIGN_SPEED      = 5

ROUTE_NONE  = 0
ROUTE_LEFT  = 1
ROUTE_RIGHT = 2
ROUTE_ANY   = 3


## Información del nodo del grafo dirigido.
class Node:
	__slots__ = [ "prev0", "prev1", "next0", "next1", "path", "route", "margin" ]
    def __init__( self ):
        ## Referencia al nodo anterior (izquierda).
        self.prev0  = None
        ## Referencia al nodo anterior (derecha).
        self.prev1  = None
        ## Referencia al nodo siguiente (izquierda).
        self.next0  = None
        ## Referencia al nodo siguiente (derecha).
        self.next1  = None
        ## Lista de puntos.
		self.path   = []
        ## Enrutamiento al salir del nodo, selecciona el siguiente nodo.
        self.route  = ROUTE_NONE
        ## Porcentaje de margen de seguridad para incorporaciones al entrar al nodo.
        self.margin = 0.0

## Puntos de la ruta de cada nodo.
class Point:
	__slots__ = [ "pos", "sign", "value" ]
    def __init__( self ):
        ## Posición del vértice.
        self.pos   = None
        ## Señal.
        self.sign  = SIGN_NONE
        ## Valor dependiente de la señal: SIGN_SEMAPHORE -> índice del tipo de semáforo, SIGN_SPEED -> máxima velocidad en Km/h
        self.value = 0


## Imprime un mensaje de error por consola.
## En caso de ser un error en la malla, los vértices en los que se detecta el problema son seleccionados.
## \note Ejecutar Blender desde la consola para ver los mensajes de error.
## \param [in] msg   Mensaje a mostrar.
## \param [in] mesh  Malla en la que se ha producido el error.
## \param [in] verts Índice de los vértices que causan el error.
def show_error( msg, mesh=None, verts=[] ):
    if mesh and verts:
        mesh.data.update()
        bpy.ops.object.select_all( action = 'DESELECT' )
        bpy.context.scene.objects.active = mesh
        mesh.select = True
        bpy.ops.object.mode_set( mode='EDIT', toggle=False ) 
        bpy.ops.mesh.select_all( action='DESELECT' ) 
        bpy.context.tool_settings.mesh_select_mode = [ True, False, False ] 
        bpy.ops.object.mode_set( mode='OBJECT', toggle=False ) 
        for i in verts: mesh.data.vertices[i].select = True            
        bpy.ops.object.mode_set( mode='EDIT', toggle=False )
    raise RuntimeError( msg )


## Construye los nodos del grafo dirigido de navegación de vehículos.
## Se sigen los siguientes pasos:
##  1. Reconocer todos los VertexGroups (señales).
##  2. Detectar instersecciónes entre aristas y crear nuevos vértices para eliminarlas (GROUP_CROSS).
##  3. Detectar las aristas demasiado largas (>MAX_EDGE_LENGTH) y subdividirlas añadiendo vértices.
##  4. Construir un grafo (no dirigido) a partir de las aristas entre vértices.
##  5. Localizar los puntos de nacimiento, que son los vértices con grupo GROUP_SPAWN.
##  6. Iniciando en los vértices de nacimiento generar un grafo dirigido recorriendo el grafo con giros limitados.
##  7. Generar el grafo inverso al grafo dirigido para calcular tanto los sucesores como los predecesores.
##  8. Crear los nodos con la información de los vértices y los dos grafos dirigidos (normal e inverso).
##  9. Recalcular los índices de los nodos de forma optimizada para recorrerlos secuencialmente.
## \param [in] mesh  Una copia de la malla de navegación de vehículos. Esta malla será modificada añadiendo/eliminando vértices y aristas.
## \return           Lista de nodos del grafo dirigido de navegación.
def build_graph( mesh ):
    
    mesh.data.update()
    edges = mesh.data.edges
    verts = mesh.data.vertices

    # create vertex groups not already existing
    for name in [ "spawn", "cross", "left", "right", "yield", "stop" ]:
        if name not in mesh.vertex_groups:
            mesh.vertex_groups.new( name )
    
    # vertex groups indexes
    group_spawn  = mesh.vertex_groups.find( "spawn"  )
    group_cross  = mesh.vertex_groups.find( "cross"  )
    group_left   = mesh.vertex_groups.find( "left"   )
    group_right  = mesh.vertex_groups.find( "right"  )
    group_yield  = mesh.vertex_groups.find( "yield"  )
    group_stop   = mesh.vertex_groups.find( "stop"   )
    group_speed  = [ mesh.vertex_groups.find( "speed"+str(i) ) for i in range(256) ]
    group_sems   = [ mesh.vertex_groups.find( "sem"+str(i) )   for i in range(256) ]
    def _group( i ):
        groups = verts[i].groups
        if len(groups) < 1: return GROUP_NONE
        if len(groups) > 1: show_error( "vertex with more than one vertex group", mesh, [i] )
        g = groups[0].group
        if g == group_spawn : return GROUP_SPAWN
        if g == group_cross : return GROUP_CROSS
        if g == group_left  : return GROUP_LEFT
        if g == group_right : return GROUP_RIGHT
        if g == group_yield : return GROUP_YIELD
        if g == group_stop  : return GROUP_STOP
        if g in group_speed : return GROUP_SPEED + group_speed.index( g )
        if g in group_sems  : return GROUP_SEMAPHORE + group_sems.index( g )
        show_error( "unknown vertex group [%d] '%s'" % ( g, mesh.vertex_groups[g] ) )

    mesh.data.update()
    edges = mesh.data.edges
    verts = mesh.data.vertices
    
    # split long edges
    for e0 in range( 0, len(edges) ):
        ( i0, i1 ) = edges[e0].vertices
        ( a0, a1 ) = ( verts[i0].co, verts[i1].co )
        nx = abs( int(a1.x/CELL_SIZE) - int(a0.x/CELL_SIZE) )
        ny = abs( int(a1.y/CELL_SIZE) - int(a0.y/CELL_SIZE) )
        n = max( nx, ny )
        if n > 1:
            verts.add( n-1 )
            for i in range( 1, n ):
                verts[-n+i].co = a0 + (i*1.0/n) * (a1-a0)
            edges.add( n-1 )
            edges[-1].vertices[0] = verts[-1].index
            edges[-1].vertices[1] = edges[e0].vertices[1]
            edges[e0].vertices[1] = verts[-n+1].index
            for i in range( 1, n-1 ):
                edges[-n+i].vertices[0] = verts[-n+i+0].index
                edges[-n+i].vertices[1] = verts[-n+i+1].index

    mesh.data.update()
    edges = mesh.data.edges
    verts = mesh.data.vertices
    
    # detect edge intersections and create GROUP_CROSS vertexes
    def _cross( e0, e1, co ):
        verts.add( 1 )
        edges.add( 2 )
        verts[-1].co = co
        edges[-1].vertices[0] = verts[-1].index
        edges[-2].vertices[0] = verts[-1].index
        edges[-1].vertices[1] = edges[e0].vertices[1]
        edges[-2].vertices[1] = edges[e1].vertices[1]
        edges[e0].vertices[1] = verts[-1].index
        edges[e1].vertices[1] = verts[-1].index
        vgroup = mesh.vertex_groups[ group_cross ]
        vgroup.add( [verts[-1].index] , 1.0, 'REPLACE' )
        print( "Added vertex 'cross', coords:", co )
    sparse = {}
    indexes = set( range( 0, len(edges) ) )
    while indexes:
        e0 = indexes.pop()
        ( i0, i1 ) = edges[e0].vertices
        ( a0, a1 ) = ( verts[i0].co, verts[i1].co )
        t = ( a0 + a1 ) / 2.0
        ( cx, cy ) = ( int(t.x/CELL_SIZE), int(t.y/CELL_SIZE) )
        if (cx,cy) not in sparse:
            sparse[ (cx,cy) ] = set()
        sparse[ (cx,cy) ].add( e0 )
        near_indexes = set()
        for y in ( cy-1, cy, cy+1 ):
            for x in ( cx-1, cx, cx+1 ):
                if (x,y) in sparse:
                    near_indexes.update( sparse[ (x,y) ] )
        for e1 in near_indexes:
            if e1 == e0: continue
            ( j0, j1 ) = edges[e1].vertices
            ( b0, b1 ) = ( verts[j0].co, verts[j1].co )
            if j0 != i0 and j0 != i1 and j1 != i0 and j1 != i1: # skip adjacent edges
                a = a1 - a0;  a.z = 0.0
                b = b1 - b0;  b.z = 0.0
                c = b0 - a0;  c.z = 0.0
                r = a.cross(b).z                     # sin(angle) between edges
                if r*r > 0.0000001:                  # parallel edges cant intersect
                    r = c.cross(b).z / r             # normalized distance from a0 to intersection point
                    if r > 0.0 and r < 1.0:          # normalized distance inside the e0 edge
                        t = (r*a.x-c.x)/b.x if b.x*b.x > b.y*b.y else (r*a.y-c.y)/b.y
                        if t > 0.0 and t < 1.0:      # normalized distance inside the e1 edge
                            _cross( e0, e1, a0+r*(a1-a0) )
                            indexes.update( [ e0, e1, len(edges)-2, len(edges)-1 ] )
                            break

    del indexes
    del sparse

    mesh.data.update()
    edges = mesh.data.edges
    verts = mesh.data.vertices
    
    # construct a graph from edges
    # vertex index --> adjacent vertex indexes
    graph = {}
    for n in range( len(edges) ):
        ( i, j ) = edges[n].vertices
        if i not in graph: graph[i] = []
        if j not in graph: graph[j] = []
        graph[i].append( j )
        graph[j].append( i )
    
    # spawn point are in 'spawn' vertex group
    # only end-points (one adjacent vertex) are supported
    spawn = [] # list of spawn point indexes
    for (i,vv) in graph.items():
        for group in verts[i].groups:
            if group.group == group_spawn:
                if len(vv) != 1: show_error( "non end-point in group 'spawn'", mesh, [i]+vv )
                spawn.append( i )
    if len(spawn) < 1: show_error( "missing 'spawn' vertexes" )
    
    # transform to a digraph simulating a driver over the road
    # driver can turn if the angle <= 60 deg
    # start driving in spawn points
    def _can_turn( i0, i1, i2 ):    # indexes to previous, current and next vertexes
        if i0 == i1 or i0 == i2: return False
        if i1 == i0 or i1 == i2: return False
        if i2 == i0 or i2 == i1: return False
        dir01 = verts[i1].co - verts[i0].co
        dir12 = verts[i2].co - verts[i1].co        
        if dir01.length < 0.01: show_error( "doubplicated verts: %d %d" % (i0,i1), mesh, (i0,i1) )
        if dir12.length < 0.01: show_error( "doubplicated verts: %d %d" % (i1,i2), mesh, (i1,i2) )
        ang = dir01.angle(dir12)
        if verts[i1].groups and verts[i1].groups[0].group == group_cross:
            return ang <= 1*math.pi/180
        return ang <= 60*math.pi/180
    digraph = {}
    stack = []              # using a stack to skip default 1000 recursion limit
    visited = set()
    for i0 in spawn:        # start simulation in spawn points
        i1 = graph[i0][0]   # spawn points only have one next point
        digraph[i0] = [ i1 ]
        stack.append( (i0,i1) )
    while stack:
        (i0,i1) = stack.pop(0)    # i0: previous vertex index, i1: current vertex index
        visited.add( (i0,i1) )
        vv = [ i2 for i2 in graph[i1] if _can_turn( i0, i1, i2 ) ]
        if i1 in digraph:
            vv = list( set( digraph[i1] + vv ) )
        digraph[i1] = vv
        for i2 in vv:
            if (i1,i2) not in visited:
                if (i2,i1) not in visited:
                    stack.append( (i1,i2) )

    del graph
    del visited
    del stack

    # find nodes and paths
	nodes = {}
	stack = [ (None,i,digraph[i][0]) for i in reverse(spawn) ]
	while stack:
		( prev, i0, i1 ) = stack_indexes.pop()
		if (i0,i1) in nodes:
			node = nodes[ (i0,i1) ]
		else:
			node = Node()
			nodes[ (i0,i1) ] = node
			node.path = [ i0 ]
			while True:
				node.path.append( i1 )
				next = digraph[i1]
				n = len( next )
				if n != 1: break
				i1 = next[0]
			if n >= 3:
				show_error( "not a binary graph", mesh, [i]+next )
			if n >= 2: stack.append( ( node, i1, next[1] ) )
			if n >= 1: stack.append( ( node, i1, next[0] ) )
		if prev:
			# dont care about left/right side (post ordering)
			if   not prev.next0 or prev.next0 == node: prev.next0 = node
			elif not prev.next1 or prev.next1 == node: prev.next1 = node
			else : show_error( "not a binary graph (direct)", mesh, [ prev.path[-1], node.path[0] ] )
			if   not node.prev0 or node.prev0 == prev: node.prev0 = prev
			elif not node.prev1 or node.prev1 == prev: node.prev1 = prev
			else : show_error( "not a binary graph (inverse)", mesh, [ prev.path[-1], node.path[0] ] )

	del digraph
	del stack
	nodes = list( nodes.values() )

	# finish nodes
	def _sort( a, b, dir ):	# sort left/right references
		if not a and not b: return ( None, None )
		if not a and     b: return (    b, None )
		if not b and     a: return (    a, None )
		if dir > 0: ( ii, aa, bb ) = ( a.path[+0], a.path[+1], b.path[+1] )
		if dir < 0: ( ii, aa, bb ) = ( a.path[-1], a.path[-2], b.path[-2] )
		( ipos, apos, bpos ) = ( verts[ii].co, verts[aa].co, verts[bb].co )
		half = 0.5*(apos+bpos) - ipos
		side = dir * half.cross( apos-ipos ).z
		return ( a, b ) if side >= 0.0 else ( b, a )
    def _route( node ):
        if not node.next0 and not node.next1: return ROUTE_NONE
        if not node.next0: return ROUTE_RIGHT
        if not node.next1: return ROUTE_LEFT
		next_group = _group( node.path[-1] )
        if next_group == GROUP_CROSS:
			if node == node.next0.prev0: return ROUTE_RIGHT
			if node == node.next0.prev1: return ROUTE_LEFT
        return ROUTE_ANY
    def _margin( node ):
        if not node.prev0: return 0.0
        if not node.prev1: return 0.0
        dir0 = verts[node.prev0.path[-1]].co - verts[node.prev0.path[-2]].co
        dir1 = verts[node.prev1.path[-1]].co - verts[node.prev1.path[-2]].co
        ang  = max( math.pi/32, abs( dir0.angle( dir1 ) ) )
        return max( 10.0, 1.0 / math.sin( ang ) )  # range = ( 1.0, 10.0 )
    for node in nodes:
		assert( not node.next0 or node.next0.path[+0] == node.path[-1] )
		assert( not node.next1 or node.next1.path[+0] == node.path[-1] )
		assert( not node.prev0 or node.prev0.path[-1] == node.path[+0] )
		assert( not node.prev1 or node.prev1.path[-1] == node.path[+0] )
		( node.next0, node.next1 ) = _sort( node.next0, node.next1, +1 )
		( node.prev0, node.prev1 ) = _sort( node.prev0, node.prev1, -1 )
		assert( not ( node.prev0 and node.prev1 ) or node.prev0.next0 == node.prev1.next0 )
		assert( not ( node.prev0 and node.prev1 ) or node.prev0.next1 == node.prev1.next1 )
		assert( not ( node.next0 and node.next1 ) or node.next0.prev0 == node.next1.prev0 )
		assert( not ( node.next0 and node.next1 ) or node.next0.prev1 == node.next1.prev1 )
        node.route  = _route( node )
        node.margin = _margin( node )

	# finish paths
    def _sign( group ):
        if group == GROUP_SPAWN : return ( SIGN_SPAWN, SIGN_SPAWN, 0 )
        if group == GROUP_LEFT  : return ( SIGN_NONE,  SIGN_YIELD, 0 )
        if group == GROUP_RIGHT : return ( SIGN_YIELD, SIGN_NONE,  0 )
        if group == GROUP_YIELD : return ( SIGN_YIELD, SIGN_YIELD, 0 )
        if group == GROUP_STOP  : return ( SIGN_STOP,  SIGN_STOP,  0 )
        if group >= GROUP_SEMAPHORE:
            return ( SIGN_SEMAPHORE, SIGN_SEMAPHORE, group-GROUP_SEMAPHORE )
        if group >= GROUP_SPEED:
            return ( SIGN_SPEED, SIGN_SPEED, group-GROUP_SPEED )
        return ( SIGN_NONE, SIGN_NONE, 0 )
    for node in nodes:
		is_right = node.next0 and node.next0.prev1 == node
		path = []
		for i in node.path:
			( sign0, sign1, value ) = _sign( _group( i ) )
			path.append( Point() )
			path[-1].pos   = verts[i].co
			path[-1].sign  = sign1 if is_right else sign0
			path[-1].value = value
		node.path = path

	# split long paths inserting nodes
	def _cell( point ):
		cx = int( point.pos.x / CELL_SIZE )
		cy = int( point.pos.y / CELL_SIZE )
		return ( cx, cy )
    for node in nodes:
		paths = [ [] ]
		old_cell = _cell( node.path[0] )
		for point in node.path:
			paths[-1].append( point )
			cell = _cell( point )
			if cell != old_cell:
				paths.append( [ point ] )
				old_cell = cell
		if len(paths) > 2:
			for i in range( 1, len(paths)-1 )
				new = Node()
				new.next0  = node.next0
				new.next1  = node.next1
				new.prev0  = node
				new.route  = node.route
				if node.next0 and node.next0.prev0 == node: node.next0.prev0 == new
				if node.next0 and node.next0.prev1 == node: node.next0.prev1 == new
				if node.next1 and node.next1.prev0 == node: node.next1.prev0 == new
				if node.next1 and node.next1.prev1 == node: node.next1.prev1 == new
				node.next0 = new
				node.next1 = None
				node.route = ROUTE_LEFT			
				new.path   = paths[i-0] + paths[i+1]
				node.path  = paths[i-1]
				node = new
	
	
	...
    # linear conversion from vertex indexes to node indexes
    linear = {}
    def _linear( idx ):
        return 0 if idx < 0 else linear[ idx ]
    for index in range( len(nodes) ):
        node = nodes[index]
        linear[ node._idx ] = index
    for index in range( len(nodes) ):
        node = nodes[index]
        node.prev0 = _linear( node.prev0 )
        node.prev1 = _linear( node.prev1 )
        node.next0 = _linear( node.next0 )
        node.next1 = _linear( node.next1 )

    # security checks
    assert( nodes[ 0 ].sign0 == SIGN_NONE )
    assert( nodes[ 0 ].sign1 == SIGN_NONE )
    for i in range( len(spawn) ):
        assert( nodes[i+1].sign0 == SIGN_SPAWN )
        assert( nodes[i+1].sign1 == SIGN_SPAWN )
        assert( nodes[i+1].prev0 == 0 )
        assert( nodes[i+1].prev1 == 0 )
    assert( nodes[ len(spawn)+1 ].sign0 != SIGN_SPAWN )
    assert( nodes[ len(spawn)+1 ].sign1 != SIGN_SPAWN )
    
    return nodes    
    

## Escribe los datos del grafo de navegación de vehículos en un fichero.
## La cabecera del fichero ocupa 32 bytes:
##  \verbatim
##  - Identificador "NAV_VEH_GRAPH".  [ char[16]  16B ] El resto de chars se rellena con '\0'.
##  - Número total de nodos.          [ uint       4B ]
##  - Número de nodos de nacimiento.  [ uint       4B ]
##  - Espacio libre de relleno.       [ uint[2]  2*4B ]
##  \endverbatim
## Cada nodo del grafo ocupa 32 bytes:
##  \verbatim
##  - sign/route (left/right)         [ byte[2]  2*1B ] El byte se divide en 4 bits para sign y 4 bits para route.
##  - semaphore index                 [ byte     1*1B ]
##  - margin dist prev nodes          [ byte     1*1B ]
##  - prev node indexes               [ uint[2]  2*4B ]
##  - next node indexes               [ uint[2]  2*4B ]
##  - position (x,y,z)                [ float[3] 3*4B ]
##  \endverbatim
## \param [in] mesh      Malla de navegación de vehículos.
## \param [in] nodes     Lista de nodos del grafo dirigido.
## \param [in] filepath  Nombre del fichero de salida.
def write_graph( mesh, nodes, filepath ):
    
    matrix = mesh.matrix_world.to_4x4()

    spawn = []
    for node in nodes[1:]:
        if node.sign0 != SIGN_SPAWN: break
        spawn.append( node )
    assert( len(spawn) > 0 )

    f = open( filepath, "wb" )

    # write file header ( offset=0B, size=32B )    
    f.write( struct.pack( "16B", *map(ord,"NAV_VEH_GRAPH\0\0\0") ) )
    f.write( struct.pack( "I", len(nodes) ) )  # number of total nodes
    f.write( struct.pack( "I", len(spawn) ) )  # number of spawn nodes
    f.write( struct.pack( "2I", 0, 0 ) )       # padding to 32B

    # write nodes data ( offset=32B, size=len(nodes)*32B )
    packer = struct.Struct( "2B B B 2I 2I 3f" )
    assert( packer.size == 32 )
    for n in nodes:
        pos = ( 0.0, 0.0, 0.0 ) if not n.pos else matrix * n.pos
        from0  = n.sign0 | ( n.route0 << 4 )
        from1 = n.sign1 | ( n.route1 << 4 )
        margin = int( 255.9 * n.margin )
        f.write( packer.pack( from0, from1, n.sem, margin, n.prev0, n.prev1, n.next0, n.next1, pos[0], pos[1], pos[2] ) )
    
    f.close()


## Crea una copia de la malla original llamada <nombre_original>_temp.
## \todo Eliminar la copia anteriror si ya existe. Actualmente se muestra un mensaje de error.
## \param [in] mesh_orig  Malla original a duplicar.
## \return                Copia independiente de la malla original.
def duplicate_mesh( mesh_orig ):

    name = mesh_orig.name + "__temp"

    if name in bpy.data.objects:
        show_error( "Mesh duplication '%s' already exists." % (name) )
        # not working !!
        #bpy.data.objects.remove( bpy.data.objects[name] )
    
    bpy.ops.object.select_all( action = 'DESELECT' )
    bpy.context.scene.objects.active = mesh_orig
    mesh_orig.select = True
    
    bpy.ops.object.duplicate()
    
    mesh = bpy.context.selected_objects[0]
    mesh.name = name

    bpy.ops.object.mode_set( mode='EDIT' ) 
    bpy.ops.mesh.select_all() 
    bpy.ops.mesh.remove_doubles( threshold=0.1 ) 
    bpy.ops.object.mode_set( mode='OBJECT' ) 

    return mesh


## Inicia el proceso de exportación de la malla de navegación a un fichero.
def export_veh_graph( context, filepath ):

    if "veh_graph" not in bpy.data.objects:
        show_error( "'veh_graph' mesh not found" )

    mesh = duplicate_mesh( bpy.data.objects["veh_graph"] )
    
    nodes = build_graph( mesh )

    #write_graph( mesh, nodes, filepath )
    
    return {'FINISHED'}



########################################################################################################
### File > Export > magv export veh graph


from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


## Operador de Blender para exportar el grafo de navegación de vehículos.
class ExportGraph( Operator, ExportHelper ):

    bl_idname    = "magv.export_veh_graph"
    bl_label     = "magv export veh graph"
    filename_ext = ".dat"
    filter_glob  = StringProperty( default="*"+filename_ext, options={'HIDDEN'} )

    def execute( self, context ):
        return export_veh_graph( context, self.filepath )


def menu_func_export( self, context ):
    self.layout.operator( ExportGraph.bl_idname, ExportGraph.bl_label )


def register():
    bpy.utils.register_class( ExportGraph )
    bpy.types.INFO_MT_file_export.append( menu_func_export )


def unregister():
    bpy.utils.unregister_class( ExportGraph )
    bpy.types.INFO_MT_file_export.remove( menu_func_export )


if __name__ == "__main__":
    print( "-"*80 )
    #register()
    #bpy.ops.magv.export_nav_graph( 'INVOKE_DEFAULT' )
    export_veh_graph( None, "/home/agv/src/magv/simulator/data/nav_veh_graph2.dat" )
    print( "DONE" )


