##
##	AUTHORS:
##		Leopoldo Armesto
##		Juan Dols
##		Jaime Molina
##
##	MAGV Simulator by Leopoldo Armesto is licensed under a Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
##	To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/deed.en_GB.
##
##  EMAIL: larmesto@idf.upv.es
##  URL: www.upv.es
##

## \namespace export_ped_graph
## Exportador para Blender del grafo de navegación de peatones.
## 
## **** PONER AQUI COMO INSTALAR EL PLUGIN EN BLENDER ****
## 
## \warning  El valor de las variables SIGN_* debe ser igual a los de nav::ped::sign.


import bpy
import math
import struct


GROUP_NONE  = 0
GROUP_SPAWN = 1
GROUP_SEMAPHORE = 1000

SIGN_NONE      = 0
SIGN_SPAWN     = 1
SIGN_SEMAPHORE = 2


## Información del nodo del grafo.
class Node:

    def __init__( self ):
		## Índice del vértice de la malla.
        self._idx   = -1
		## Posición del vértice.
        self.pos    = None
		## Señal en este nodo.
        self.sign   = 0
		## Semaforo en este nodo.
        self.sem    = 0
		## Hasta cuatro índices a nodos adyacentes.
        self.next   = ( -1, -1, -1, -1 )
		## Ángulo de los cuatro nodos adyacentes.
        self.ang    = (  0,  0,  0,  0 )


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


## Construye los nodos del grafo de navegación de peatones.
## Se sigen los siguientes pasos:
##  1. Reconocer todos los VertexGroups (señales).
##  2. Construir un grafo (no dirigido) a partir de las aristas entre vértices.
##  3. Localizar los puntos de nacimiento, que son los vértices con grupo GROUP_SPAWN.
##  4. Iniciando en los vértices de nacimiento generar un grafo no dirigido y con ciclos.
##  5. Crear los nodos con la información de los vértices y el grafo.
##  6. Recalcular los índices de los nodos de forma optimizada para recorrerlos secuencialmente.
## \param [in] mesh  Una copia de la malla de navegación de peatones. Esta malla será modificada añadiendo/eliminando vértices y aristas.
## \return           Lista de nodos del grafo de navegación.
def build_graph( mesh ):
    
    mesh.data.update()
    edges = mesh.data.edges
    verts = mesh.data.vertices

    # create vertex groups not already existing
    for name in [ "spawn" ]:
        if name not in mesh.vertex_groups:
            mesh.vertex_groups.new( name )

    # vertex groups indexes
    group_spawn  = mesh.vertex_groups.find( "spawn"  )
    group_sems   = [ mesh.vertex_groups.find( "sem"+str(i) ) for i in range(256) ]
    def _group( i ):
        groups = verts[i].groups
        if len(groups) < 1: return GROUP_NONE
        if len(groups) > 1: show_error( "vertex with more than one vertex group", mesh, [i] )
        g = groups[0].group
        if g == group_spawn : return GROUP_SPAWN
        if g in group_sems  : return GROUP_SEMAPHORE + group_sems.index( g )
        show_error( "unknown vertex group [%d] '%s'" % ( g, mesh.vertex_groups[g] ) )
    
    # construct a graph from edges
    # vertex index --> adjacent vertex indexes
    graph = {}
    for n in range( len(edges) ):
        ( i, j ) = edges[n].vertices
        if i not in graph: graph[i] = []
        if j not in graph: graph[j] = []
        graph[i].append( j )
        graph[j].append( i )
        if len( graph[i] ) > 4: show_error( "more than 4 adjacent vertexes", mesh, [i] )
        if len( graph[j] ) > 4: show_error( "more than 4 adjacent vertexes", mesh, [j] )
    
    # spawn point are in 'spawn' vertex group
    # only end-points (one adjacent vertex) are supported
    spawn = [] # list of spawn point indexes
    for (i,vv) in graph.items():
        for group in verts[i].groups:
            if group.group == group_spawn:
                if len(vv) != 1: show_error( "non end-point in group 'spawn'", mesh, [i]+vv )
                spawn.append( i )
    if len(spawn) < 1: show_error( "missing 'spawn' vertexes" )
    
    # build graph nodes
    nodes = [ Node() ]
    def _sign( group ):
        if group == GROUP_SPAWN : return ( SIGN_SPAWN, 0 )
        if group >= GROUP_SEMAPHORE:
            return ( SIGN_SEMAPHORE, group-GROUP_SEMAPHORE )
        return ( SIGN_NONE, 0 )
    def _angles( i1, next ):
        ret = []
        for i2 in next:
            if i2 < 0:
                ret.append( 0.0 )
            else:
                dir = verts[i2].co - verts[i1].co
                ret.append( math.atan2( dir.y, dir.x ) )
        return tuple( ret )
    def _add_node( i1 ):
        group = _group( i1 )
        node = Node()
        node._idx = i1
        node.pos  = verts[i1].co
        node.next = tuple( graph[i1] + [-1,-1,-1,-1] )[0:4]
        node.ang  = _angles( i1, node.next )
        ( node.sign, node.sem ) = _sign( group )
        nodes.append( node )
    stack = []              # using a stack to skip default 1000 recursion limit
    for i1 in spawn:
        _add_node( i1 )
        stack.extend( graph.pop(i1) )
    while stack:
        i1 = stack.pop()
        if i1 in graph:
            _add_node( i1 )
            stack.extend( graph.pop(i1) )
    
    del stack
    del graph

    # linear conversion from vertex indexes to node indexes
    linear = {}
    def _linear( idx ):
        return 0 if idx < 0 else linear[ idx ]
    for index in range( len(nodes) ):
        node = nodes[index]
        linear[ node._idx ] = index
    for index in range( len(nodes) ):
        node = nodes[index]
        node.next = ( _linear(node.next[0]), _linear(node.next[1]), _linear(node.next[2]), _linear(node.next[3]) )

    # security checks
    assert( nodes[ 0 ].sign == SIGN_NONE )
    for i in range( len(spawn) ):
        assert( nodes[i+1].sign == SIGN_SPAWN )
    assert( nodes[ len(spawn)+1 ].sign != SIGN_SPAWN )
    
    return nodes    


## Escribe los datos del grafo de navegación de peatones en un fichero.
## La cabecera del fichero ocupa 32 bytes (ver nav::ped::Header):
##  \verbatim
##  - Identificador "NAV_PED_GRAPH".  [ char[16]  16B ] El resto de chars se rellena con '\0'.
##  - Número total de nodos.          [ uint       4B ]
##  - Número de nodos de nacimiento.  [ uint       4B ]
##  - Espacio libre de relleno.       [ uint[2]  2*4B ]
##  \endverbatim
## Cada nodo del grafo ocupa 32 bytes (ver nav::ped::Node):
##  \verbatim
##  - sign                            [ byte     1*1B ]
##  - semaphore index                 [ byte     1*1B ]
##  - padding                         [ byte     1*1B ]
##  - count next nodes                [ byte     1*1B ]
##  - next/angle adyacent nodes       [ uint[4]  4*4B ] Cada uint se descompone en 8 bits (LSB) para codificar el ángulo y 24 bits (MSB) para el índice del siguiente nodo.
##  - position (x,y,z)                [ float[3] 3*4B ]
##  \endverbatim
## \param [in] mesh      Malla de navegación de peatones.
## \param [in] nodes     Lista de nodos del grafo.
## \param [in] filepath  Nombre del fichero de salida.
def write_graph( mesh, nodes, filepath ):
    
    matrix = mesh.matrix_world.to_4x4()

    spawn = []
    for node in nodes[1:]:
        if node.sign != SIGN_SPAWN: break
        spawn.append( node )
    assert( len(spawn) > 0 )

    f = open( filepath, "wb" )

    # write file header ( offset=0B, size=32B )    
    f.write( struct.pack( "16B", *map(ord,"NAV_PED_GRAPH\0\0\0") ) )
    f.write( struct.pack( "I", len(nodes) ) )  # number of total nodes
    f.write( struct.pack( "I", len(spawn) ) )  # number of spawn nodes
    f.write( struct.pack( "2I", 0, 0 ) )       # padding to 32B

    # write nodes data ( offset=32B, size=len(nodes)*32B )
    packer = struct.Struct( "1B 3B 4I 3f" )
    assert( packer.size == 32 )
    def _nextang( n, i ):
        if n.next[i] <= 0: return 0
        a = 256 + int( 128*n.ang[i]/math.pi )
        return ( n.next[i] << 8 ) | ( a & 0xFF )
    for n in nodes:
        pos = ( 0.0, 0.0, 0.0 ) if not n.pos else matrix * n.pos
        count = 4 - n.next.count( 0 );
        f.write( packer.pack( n.sign, n.sem, 0, count, _nextang(n,0), _nextang(n,1), _nextang(n,2), _nextang(n,3), pos[0], pos[1], pos[2] ) )
    
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
def export_ped_graph( context, filepath ):

    if "ped_graph" not in bpy.data.objects:
        show_error( "'ped_graph' mesh not found" )

    mesh = duplicate_mesh( bpy.data.objects["ped_graph"] )
    
    nodes = build_graph( mesh )

    write_graph( mesh, nodes, filepath )
    
    return {'FINISHED'}



########################################################################################################
### File > Export > magv export ped graph


from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


## Operador de Blender para exportar el grafo de navegación de vehículos.
class ExportGraph( Operator, ExportHelper ):

    bl_idname    = "magv.export_ped_graph"
    bl_label     = "magv export ped graph"
    filename_ext = ".dat"
    filter_glob  = StringProperty( default="*"+filename_ext, options={'HIDDEN'} )

    def execute( self, context ):
        return export_ped_graph( context, self.filepath )


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
    #bpy.ops.magv.export_ped_graph( 'INVOKE_DEFAULT' )
    export_ped_graph( None, "/home/agv/src/magv/simulator/data/nav_ped_graph.dat" )
    print( "DONE" )

    
