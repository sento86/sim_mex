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
	namespace veh
	{
		/// Cabecera del fichero de navegación de vehículos.
		struct Header {
			char			magic[16];		///< Identificador del archivo: "NAV_VEH_GRAPH" 
			unsigned int	num_nodes;		///< Número de nodos en el grafo. \warning El primer nodo de índice 0 no es válido.
			unsigned int	num_spawn;		///< Número nodos de nacimiento.  \warning El primer nodo de índice 0 no es válido.
			unsigned int	_padding[2];	///< Padding para alcanzar los 32 bytes.
		};

		/// Información sobre el estado de cada nodo del grafo.
		/// Se guarda información sobre las trayectorias o rutas de los vehículos que circulan por estos nodos,
		/// de modo que se puede determinar la intersección con la ruta de algún otro vehículo.
		struct PlanNode {
			Plan			*plan;		///< Plan con ruta pasando sobre este nodo.
			unsigned int	tick;		///< Para conocer si el nodo es válido. Ver nav::veh::tick.
			unsigned int	visited;	///< Evitar bucles o finalizar recursiones. Ver nav::veh::visited.
			float			dist;		///< Distancia que tiene que recorrer el vehículo para llegar al nodo.
			float			time;		///< Tiempo que tarda el vehículo en llegar a este nodo.
		};

		/// Contador utilizado para determinar si la información de los PlanNode está obsoleta.
		/// Los planificadores modifican la información de los nodos de plan y actualizan el contador (PlanNode::tick = veh::tick+N). \n
		/// Este contador es incrementado una vez por frame, por tanto, sumar N al contador es equivalente a decir que el nodo está reservado por un plan durante los próximos N frames. \n
		/// Cualquier nodo con un tick menor a veh::tick está obsoleto y su información no es válida.
		static unsigned int tick;
		
		/// Para indicar que un nodo ha sido visitado anteriormente.
		/// Utilizado para recorrer el grafo y no entrar en bucles. \n
		/// Antes de iniciar el recorrido por el grafo incrementamos este valor. Si PlanNode::visited es igual a veh::visited, el nodo ya lo hemos visitado, de lo contrario le asignameos este valor y lo procesamos.
		static unsigned int visited;
	}
}


const nav::veh::Graph * nav::veh::Load( const char *file )
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

	if( strcmp( header.magic, "NAV_VEH_GRAPH" ) ) {
		goto load_error;
	}
	
	r = header.num_nodes * ( sizeof(Node) + 2*sizeof(PlanNode) );
	graph = (Graph*) ::malloc( sizeof(Graph) + r );	
	if( !graph ) {
		goto load_error;
	}

	graph->num_nodes  = header.num_nodes;
	graph->num_spawns = header.num_spawn;
	graph->nodes      = (Node*) ( graph + 1 );
	graph->pnodes     = (PlanNode(*)[2]) ( graph->nodes + header.num_nodes );
	memset( graph+1, 0, r );

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


void nav::veh::Free( const nav::veh::Graph *&graph )
{
	::free( (void*)graph );
	graph = NULL;
}


const nav::veh::Node * nav::veh::GetRespawnNode( const nav::veh::Graph *graph, const int index_spawn )
{
	return &graph->nodes[ 1 + index_spawn % graph->num_spawns ]; // skip first node
}


const nav::veh::Node * nav::veh::Plan::Respawn( const nav::veh::Graph *graph, const float speed, const int index_spawn )
{
	const int index = ( index_spawn < 0 ? this->bits : index_spawn );
	
	this->graph = graph;
	this->prev  = 1 + index % graph->num_spawns;		// skip first node
	this->curr  = graph->nodes[ this->prev ].next[0];
	this->speed_limit_kmh = ( speed > 70.0f ? 255 : speed*(60*60/1000.0f) );
	
	return &graph->nodes[ this->prev ];
}


/// Marca recursivamente los nodos ocupados por un plan. Ver MarkOwnNodes().
/// \param [in]     x		 Coordeanda X de la posición del vehículo.
/// \param [in]     y        Coordeanda Y de la posición del vehículo.
/// \param [in]     length2  Distancia al cuadrado desde la posición del vihículo al nodo actual.
/// \param [in,out] plan     Plan del vehículo a insertar en los nodos de planificación.
/// \param [in]     curr     Índice del nodo actual.
/// \param [in]     forward  Indica si se está recorriendo el grafo hacia delante o hacia atrás.
static void MarkOwnNodesRecursive( const float x, const float y, const float length2, nav::veh::Plan *plan, unsigned int curr, bool forward )
{
	const nav::veh::Node &node = plan->graph->nodes[ curr ];
	nav::veh::PlanNode  *pnode = plan->graph->pnodes[ curr ];

	pnode[0].visited = nav::veh::visited;

	const float rx = node.x - x;
	const float ry = node.y - y;
	const float rr = rx*rx + ry*ry;
	if( rr > length2 ) return;
	
	//const float dist = rr * ( forward ? 0.001f : 0.000001f ); // using low values to prevent override other vehicle preferences
	const float dist = rr * ( forward ? 0.001f : 0.00001f ); // using low values to prevent override other vehicle preferences
	if( pnode[0].plan == plan || pnode[0].tick < nav::veh::tick || dist < pnode[0].dist )
	{
//		if( !forward ) {
			pnode[0].plan = plan;
			pnode[0].tick = nav::veh::tick + 1;
			pnode[0].dist = dist;
			pnode[0].time = dist;
			pnode[1] = pnode[0];
//		}

		// recursively process all not already visited nodes
		if( plan->graph->pnodes[ node.next[0] ][0].visited != nav::veh::visited ) MarkOwnNodesRecursive( x, y, length2, plan, node.next[0], forward );
		if( plan->graph->pnodes[ node.next[1] ][0].visited != nav::veh::visited ) MarkOwnNodesRecursive( x, y, length2, plan, node.next[1], forward );
		if( plan->graph->pnodes[ node.prev[0] ][0].visited != nav::veh::visited ) MarkOwnNodesRecursive( x, y, length2, plan, node.prev[0], false );
		if( plan->graph->pnodes[ node.prev[1] ][0].visited != nav::veh::visited ) MarkOwnNodesRecursive( x, y, length2, plan, node.prev[1], false );
	}
}


/// Un vehículo ocupa varios nodos del grafo dirigido a su alrededor.
/// El plan del vehículo se inserta en los nodos cercanos que no estén ocupados por otro plan de mayor prioridad.
/// \param [in]     x		 Coordeanda X de la posición del vehículo.
/// \param [in]     y        Coordeanda Y de la posición del vehículo.
/// \param [in]     length   Distancia máxima o radio de ocupación del vehículo.
/// \param [in,out] plan     Plan del vehículo a insertar en los nodos de planificación.
static void MarkOwnNodes( const float x, const float y, const float length, nav::veh::Plan *plan )
{
	nav::veh::visited++;
	plan->graph->pnodes[0][0].visited = nav::veh::visited;
	plan->graph->pnodes[0][1].visited = nav::veh::visited;

	MarkOwnNodesRecursive( x, y, length*length, plan, plan->prev, false );
	
	// inconditionally mark current node as  occupied by this car
	nav::veh::PlanNode *pnode = plan->graph->pnodes[ plan->prev ];
	pnode[0].plan = plan;
	pnode[0].tick = nav::veh::tick + 1;
	pnode[0].time = 0.0f;
	pnode[0].dist = 0.0f;
	pnode[1] = pnode[0];
}

/// .\n
/// La planificación se realiza recorriendo los nodos por los que pasará el vehículo durante los futuros \a time segundos suponiendo una velocidad media de \a speed m/s. \n
/// Podemos saber las próximas 32 direcciones de giro del vehículo (ver veh::Plan::GetTurnDirection()) y predecir su ruta sobre el grafo. \n
/// Una colisión se produce cuando la ruta de un plan intersecta con otra ruta de mayor preferencia. La preferencia se determina según la distancia o tiempo al que se encuentran los vehículos del nodo donde colisionan.
/// Si la colisión se produce por el mismo camino (un vehiculo delante sobre la misma carretera) se utiliza una preferencia basada en distancia, de modo que el vehículo de delante simpre tendrá preferencia y el atrasado obtendrá los datos de la futura colisión para frenar o actuar según le convenga.
/// En el caso de un cruce o intersección entre dos caminos, se utiliza una preferencia basada en tiempo, informando de colisión al que llegue más tarde al nodo, que debería frenar para dejar pasar al que llega primero. \n
/// La preferencia por defecto se puede alterar mediante señales de tráfico (nav::veh::sign) y semáforos (nav::sem). Al encotrarse un semáforo en rojo, la preferencia se termina y se corta la planificación resultando una colisión.
/// \note Cuando se detecta una señal de ceda el paso (veh::sign::YIELD) se utiliza el truco de escalar el tiempo de llegada del vehículo por un valor pequeño, perdiendo preferencia en los cruces.
/// \warning La señal de stop (veh::sign::STOP) no está aún implementada y se trata igual a una señal de ceda el paso.
const nav::veh::Node * nav::veh::Plan::Planify( const float x, const float y, const float length, const float speed, const float time, Info *const info )
{
	const float speed_inv = ( speed < 1.0f ? 1.0f : 1.0f/speed );
	
	const nav::veh::Node *curr_node, *prev_node;
	unsigned int way, curr, prev, target, turn_count;
	nav::veh::PlanNode *plan_node;
	bool preference, advance;
	float rx, ry, r, t;
	float yield;

	assert( info );
	info->node = NULL;
	info->plan = NULL;
	info->semaphore = 0;

	prev = this->prev;
	curr = this->curr;
	turn_count = 0;
	target = curr;

	// initial distance and time from the vehicle to its current node
	curr_node = &this->graph->nodes[ curr ];
	prev_node = &this->graph->nodes[ prev ];
	rx = curr_node->x - x;
	ry = curr_node->y - y;
	r  = sqrt( rx*rx + ry*ry );
	t  = r * speed_inv;

	assert( curr == prev_node->next[0] || curr == prev_node->next[1] );
	//way = ( prev == curr_node->prev[1] ? 1 : 0 );
	//yield = ( curr_node->sign[way] == nav::veh::sign::YIELD ? 10.0f : 1.0f );

	// check if vehicle has to advance to next node ( overpassed current node ? )
	advance = ( r < length && rx*(curr_node->x-prev_node->x) + ry*(curr_node->y-prev_node->y) < 0.0f );

	yield = 1.0f;
	
	if( curr_node->from[0].sign == nav::veh::sign::SPEED ) {
		assert( curr_node->from[1].sign == nav::veh::sign::SPEED );
		this->speed_limit_kmh = curr_node->semaphore;
	}
	
	while( curr )
	{
		assert( curr != prev );
		
		assert( prev == curr_node->prev[0] || prev == curr_node->prev[1] );
		way = ( prev == curr_node->prev[1] ? 1 : 0 ); // vehicle comes from left=0 or right=1
		
		plan_node = &this->graph->pnodes[curr][way]; // preference on my way : distance based
		preference = ( plan_node->plan == this || plan_node->tick < nav::veh::tick || ( r < plan_node->dist ) );
		if( preference )
		{
			plan_node->plan = this;
			plan_node->tick = nav::veh::tick + 1;
			plan_node->dist = r;
			plan_node->time = t*yield;

			if( curr_node->prev[1] ) {
				plan_node = &this->graph->pnodes[curr][!way]; // preference on cross : time based
				preference = ( plan_node->plan == this || plan_node->tick < nav::veh::tick || ( t*yield < plan_node->time ) );
			}
		}

		if( !preference ) {	// the plan has no preference, return information of the future collision
			info->node  = curr_node;
			info->plan  = plan_node->plan;
			info->dist  = r;
			info->time  = t;
			info->myway = ( plan_node == &this->graph->pnodes[curr][way] );
		}

		switch( curr_node->from[way].route )	// precalculated routing, choose next node
		{
			case nav::veh::route::NONE:
				assert( curr_node->next[0] == 0 && curr_node->next[1] == 0 );
				way = 0;
			break;

			case nav::veh::route::LEFT:
				assert( curr_node->next[0] != 0 );
				way = 0;
			break;

			case nav::veh::route::RIGHT:
				assert( curr_node->next[1] != 0 );
				way = 1;
			break;

			case nav::veh::route::ANY:
				assert( curr_node->next[0] != 0 && curr_node->next[1] != 0 );
				way = this->GetTurnDirection( turn_count++ );
			break;

			default:
				assert( !"veh::Plan::Planify: Invalid route" );
		}

		switch( curr_node->from[way].sign )
		{
			case nav::veh::sign::NONE:
			case nav::veh::sign::SPAWN:
			break;
			
			case nav::veh::sign::YIELD:
				yield = 10.0f;
			break;
			
			case nav::veh::sign::STOP:
				//####TODO
				yield = 10.0f;
			break;
			
			case nav::veh::sign::SEMAPHORE:
				assert( curr_node->semaphore );
				if( preference ) {
					if( !nav::sem::IsGreen( curr_node->semaphore ) ) {
						preference = false;
						info->node  = curr_node;
						info->dist  = r;
						info->time  = t;
						info->myway = 1;
						info->semaphore = curr_node->semaphore;
					}
				}
			break;
			
			case nav::veh::sign::SPEED:
				assert( curr_node->semaphore );
				if( curr_node->semaphore < this->speed_limit_kmh )
					this->speed_limit_kmh = curr_node->semaphore;
			break;
			
			default:
				assert( !"nav::veh::Plan::Planify: Invalid curr_node->sign[way]" );
		}
		
		prev = curr;
		curr = curr_node->next[way];

		if( r < length ) {
			target = curr;
		}

		if( advance ) {			// vehicle overpassed current node, aim to next node
			advance = false;
			this->prev = prev;
			this->curr = curr;
			if( turn_count ) {
				assert( turn_count == 1 );
				turn_count = 0;
				this->Turn();
			}
		}

		if( !preference || t > time ) break;
		
		// follow the path measuring distance and time
		curr_node = &this->graph->nodes[ curr ];
		prev_node = &this->graph->nodes[ prev ];
		rx = curr_node->x - prev_node->x;
		ry = curr_node->y - prev_node->y;
		r += sqrt( rx*rx + ry*ry );
		t  = r * speed_inv;
	}
	
	MarkOwnNodes( x, y, length, this );

	// fast way to evaluate the curvature of the path:  ( dist( node_first, node_last )^2 / path_length^2 )^2   // from 0.0 to 1.0=line
	rx = curr_node->x - x;
	ry = curr_node->y - y;
	info->curvature = ( rx*rx + ry*ry ) / ( r*r );
	info->curvature *= info->curvature;

	info->speed_limit = this->speed_limit_kmh * (1000.0f/60/60);	// km/h to m/s

	return ( target ? &this->graph->nodes[target] : NULL );
}


void NearbyRecursive( nav::veh::Plan::NearbyCallback callback, nav::veh::Plan &plan, const unsigned int curr, const unsigned int prev, float dist )
{
	const nav::veh::Node &node = plan.graph->nodes[curr];
	
	plan.graph->pnodes[curr][0].visited = nav::veh::visited;	// mark current node as visited to cut recursion
	
	if( prev ) { // substract distance, cut recursion when dist < 0
		const float rx = node.x - plan.graph->nodes[prev].x;
		const float ry = node.y - plan.graph->nodes[prev].y;
		dist -= sqrt( rx*rx + ry*ry );
	}

	for( int i = 0; i < 2; i++ ) {
		if( node.next[i] ) {
			nav::veh::PlanNode *pnode = plan.graph->pnodes[ node.next[i] ];						// next node to visit
			if( pnode[0].visited != nav::veh::visited ) {										// check node not already visited
				for( int way = 0; way < 2; way++ ) {											// 
					if( pnode[way].plan != &plan && pnode[way].tick >= nav::veh::tick ) {		// is another plan over this node ?
						if( pnode[way].dist == 0.0f && pnode[way].plan ) {						// is the vehicle exactly over this node ?
							callback( &plan, pnode[way].plan );									// notify the intersection
						}						
					}
				}
				if( dist > 0.0f ) {
					NearbyRecursive( callback, plan, node.next[i], curr, dist );					
				}
			}
		}
	}
}

void nav::veh::Plan::Nearby( const float dist, NearbyCallback callback )
{
	nav::veh::visited++;
	NearbyRecursive( callback, *this, this->curr, 0, dist );
}


void nav::veh::Initialize( void )
{
	nav::veh::tick    = 1;
	nav::veh::visited = 1;
}


void nav::veh::Finalize( void )
{
	nav::veh::tick    = 0;
	nav::veh::visited = 0;
}


void nav::veh::Update( const float dt )
{
	nav::veh::tick++;
}

