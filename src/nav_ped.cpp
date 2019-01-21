/*
**	AUTHORS:
**		Leopoldo Armesto
**		Juan Dols
**		Jaime Molina
**
**	MAGV Simulator by Leopoldo Armesto is licensed under a Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
**	To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/deed.en_GB.
**
**  EMAIL: larmesto@idf.upv.es
**  URL: www.upv.es
*/


/// \cond PRIVATE


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include "nav.hxx"



namespace nav
{	
	namespace ped
	{
		/// Cabecera del fichero de navegación de peatones.
		struct Header {
			char			magic[16];		///< Identificador del archivo: "NAV_PED_GRAPH" 
			unsigned int	num_nodes;		///< Número de nodos en el grafo. \warning El primer nodo de índice 0 no es válido.
			unsigned int	num_spawn;		///< Número nodos de nacimiento.  \warning El primer nodo de índice 0 no es válido.
			unsigned int	_padding[2];	///< Padding para alcanzar los 32 bytes.
		};
	}
}


const nav::ped::Graph * nav::ped::Load( const char *file )
{
	FILE     *f      = NULL;
	Graph    *graph  = NULL;
	Header   header;
	size_t   r;

	assert( sizeof(Header) == 32 );
	assert( sizeof(Node) == 32 );

	f = fopen( file, "rb" );
	if( !f ) {
		goto load_error;
	}

	r = fread( &header, sizeof(header), 1, f );
	if( r != 1 ) {
		goto load_error;
	}

	if( strcmp( header.magic, "NAV_PED_GRAPH" ) ) {
		goto load_error;
	}
	
	graph = (Graph*) ::malloc( sizeof(Graph) + header.num_nodes * sizeof(Node) );	
	if( !graph ) {
		goto load_error;
	}

	graph->num_nodes  = header.num_nodes;
	graph->num_spawns = header.num_spawn;
	graph->nodes      = (Node*) ( graph + 1 );	

	r = fread( graph->nodes, sizeof(Node), header.num_nodes, f );
	if( r != header.num_nodes ) {
		goto load_error;
	}
	
	fclose( f );
	f = NULL;
		
	//####TODO: endianess
	
	return graph;
	
load_error:
	if( f ) fclose( f );
	if( graph ) ::free( (void*)graph  );
	return NULL;
}


void nav::ped::Free( const nav::ped::Graph *&graph )
{
	::free( (void*)graph );
	graph = NULL;
}


const nav::ped::Node * nav::ped::GetRespawnNode( const nav::ped::Graph *graph, const int index_spawn )
{
	return &graph->nodes[ 1 + index_spawn % graph->num_spawns ]; // skip first node
}


const nav::ped::Node * nav::ped::Plan::Respawn( const nav::ped::Graph *graph, const int index_spawn )
{
	this->graph = graph;
	this->curr  = 1 + index_spawn % graph->num_spawns;	// skip first node
	this->prev  = this->curr;
	return &graph->nodes[ this->curr ];
}


/// Selecciona uno de los posibles nodos adyacentes.
/// La elección se realiza probabilistacamente dando mas prioridad a los nodos frontales, es decir, la probabilidad es proporcional al ángulo de giro del peatón hacia el nodo.
/// En lugar de utilizar la dirección del peaton, se utiliza la dirección contraria ( \a ang + 180º ), de modo que a mayor ángulo (delante) con la dirección del nodo, es más probable su elección.
/// \param [in] plan  Plan del peatón.
/// \param [in] ang   Ángulo de dirección del peatón.
/// \param [in] node  Nodo actual en el que se encuentra el peatón.
/// \return           Índice del nodo adyacente resultante.
static int ChooseNext( const nav::ped::Plan &plan, const float ang, const nav::ped::Node &node )
{
	// inverse direction, byte normalized: 0=256=2PI, 128=PI, 64=PI/2
	const int ang256 = 0xFF & (int) ( 128 + 256 * ang/(2*M_PI) );
	
	switch( node.count )
	{
		case 1: { // choose the other node
			assert( node.na[0].next && !node.na[1].next && !node.na[2].next && !node.na[3].next );
			return node.na[0].next;
		}
		
		case 2: { // choose the unvisited node
			assert( node.na[0].next && node.na[1].next && !node.na[2].next && !node.na[3].next );
			const unsigned int next0 = node.na[0].next;
			const unsigned int next1 = node.na[1].next;
			return ( next0 == plan.prev ? next1 : next0 );
		}
		
		case 3: { // randomly choose next, angle-based probability
			assert( node.na[0].next && node.na[1].next && node.na[2].next && !node.na[3].next );
			const int diff0 = abs( ang256 - node.na[0].ang );
			const int diff1 = abs( ang256 - node.na[1].ang );
			const int diff2 = abs( ang256 - node.na[2].ang );
			const int prob0 = ( diff0 > 128 ? 256-diff0 : diff0 );
			const int prob1 = ( diff1 > 128 ? 256-diff1 : diff1 );
			const int prob2 = ( diff2 > 128 ? 256-diff2 : diff2 );
			int r = (int) ( plan.GetRandom() % ( prob0 + prob1 + prob2 ) );
			r -= prob0; if( r <= 0 ) return node.na[0].next;
			r -= prob1; if( r <= 0 ) return node.na[1].next;
			r -= prob2; if( r <= 0 ) return node.na[2].next;
			assert( !"ChooseNext: sould not happen" );
		}
		
		case 4: { // randomly choose next, angle-based probability
			assert( node.na[0].next && node.na[1].next && node.na[2].next && node.na[3].next );
			const int diff0 = abs( ang256 - node.na[0].ang );
			const int diff1 = abs( ang256 - node.na[1].ang );
			const int diff2 = abs( ang256 - node.na[2].ang );
			const int diff3 = abs( ang256 - node.na[3].ang );
			const int prob0 = ( diff0 > 128 ? 256-diff0 : diff0 );
			const int prob1 = ( diff1 > 128 ? 256-diff1 : diff1 );
			const int prob2 = ( diff2 > 128 ? 256-diff2 : diff2 );
			const int prob3 = ( diff3 > 128 ? 256-diff3 : diff3 );
			int r = (int) ( plan.GetRandom() % ( prob0 + prob1 + prob2 + prob3 ) );
			r -= prob0; if( r <= 0 ) return node.na[0].next;
			r -= prob1; if( r <= 0 ) return node.na[1].next;
			r -= prob2; if( r <= 0 ) return node.na[2].next;
			r -= prob3; if( r <= 0 ) return node.na[3].next;
			assert( !"ChooseNext: sould not happen" );
		}
		
		default:
			assert( !"ChooseNext: Invalid node.count" );
	}
	
	return 0;
}


/// .\n
/// La planificación de un peatón es bastante simple. Al alcanzar cierta distancia al nodo destino, se actualiza el destino a otro nodo adyacente del grafo. Así sucesivamente. \n
/// La elección del siguiente nodo destino se realiza aleatoriamente, dando mayor o menor probabilidad a los nodos en función del ángulo de giro del peatón.
/// Se asigna mayor probabiidad a los nodos que el peatón tiene delante y una probabilidad menor cuando los giros son grandes.
const nav::ped::Node * nav::ped::Plan::Planify( const float x, const float y, const float ang, const float distance )
{
	if( this->curr == 0 ) return NULL;

	const nav::ped::Node &node = this->graph->nodes[ this->curr ];
	const float rx = node.x - x;
	const float ry = node.y - y;
	
	if( rx*rx + ry*ry < distance*distance )
	{
		const int next = ChooseNext( *this, ang, node );
		
		this->prev = this->curr;
		this->curr = next;
	}

	return ( this->curr ? &this->graph->nodes[this->curr] : NULL );
}


const nav::ped::Node * nav::ped::Plan::RePlanify( const float ang )
{
	if( this->curr == 0 ) return NULL;
	
	const int next = ChooseNext( *this, ang+M_PI, this->graph->nodes[this->curr] );
	if( next ) {
		this->prev = this->curr;
		this->curr = next;
	}
	
	return &graph->nodes[ this->curr ];
}


void nav::ped::Initialize( void )
{
	// nothing to do
}


void nav::ped::Finalize( void )
{
	// nothing to do
}


void nav::ped::Update( const float dt )
{
	// nothing to do
}



