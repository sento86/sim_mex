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

/// \file
/// .\n
/// Como se indica en ::world, el mundo se divide en celdas de tamaño world::CELL_SIZE. \n
/// Las entidades se insertan/extraen en su celda correspondiente calculada en función de su posición (x,y). Ver ::CellHashPos(). \n
/// Estas celdas en realidad son listas enlazadas. Cada entidad dispone de un puntero a la entidad siguiente (world::Entity::next) para formar esta listas. \n
/// Las celdas se mantienen organizadas en una cuadricula dispersa formado por el mapa ::cells : \a índice_de_celda --> \a lista_de_entidades.


#include <stdlib.h>
#include <assert.h>
#include <map>

#include "world.hpp"


typedef std::map< unsigned int, world::Entity* > CellMap;	///< Mapa de índices a celdas.

static world::Entity *queueHead;
static world::Entity *queueTail;

static CellMap  cells;		///< Cuadricula dispersa de celdas.
static int		entities;	///< Número de entidades en las celdas.

#define CellFromFloat( f ) ( (unsigned short) ( 0x7FFF + (f)*CELL_SIZE_INV ) )		///< Conversión de \a float a \a coordenada_de_celda.
#define CellToFloat( c )   ( (float) ( ( (c) - 0x7FFF + 0.5f ) * CELL_SIZE ) )		///< Conversión de \a coordenada_de_celda a \a float.
#define CellHash( cx, cy ) ( (unsigned int) ( ( (cy) << 16 ) | ( (cx) << 0 ) ) )	///< Conversión de \a coordenada_de_celda (x,y) a \a índice_de_celda.
#define CellHashPos( fx, fy )  CellHash( CellFromFloat(fx), CellFromFloat(fy) )		///< Conversión de posición de entidad (x,y) a \a índice_de_celda.



bool world::Initialize( void )
{
	world::Finalize();
	return true;
}


void world::Finalize( void )
{
	for( CellMap::iterator it = cells.begin(); it != cells.end(); ++it )
	{
		world::Entity *ent, *next = it->second;
		while( next )
		{
			ent  = next;
			next = ent->next;
			
			ent->cell_x = 0;
			ent->cell_y = 0;
			ent->next   = NULL;
		}
	}

	world::Entity *ent;
	while( queueHead )
	{
		ent       = queueHead;
		queueHead = ent->next;
		
		ent->cell_x = 0;
		ent->cell_y = 0;
		ent->next   = NULL;
	}
	
	cells.clear();
	entities = 0;

	queueHead = NULL;
	queueTail = NULL;
}


void world::QueuePushBack( world::Entity *ent )
{
	assert( (bool)queueHead == (bool)queueTail );
	
	if( ent->cell_x | ent->cell_y ) ent->WorldDelete();
	
	if( queueHead )
	{
		queueTail->next = ent;
		queueTail = ent;
	}
	else
	{
		queueHead = ent;
		queueTail = ent;
	}
}

	
world::Entity * world::QueuePopFront( void )
{
	assert( (bool)queueHead == (bool)queueTail );

	Entity *ent = queueHead;
	if( queueHead == queueTail )
	{
		if( !ent ) return NULL;
		queueHead = NULL;
		queueTail = NULL;
	}
	else
	{
		queueHead = ent->next;
	}

	assert( !ent->cell_x && !ent->cell_y );

	ent->next = NULL;

	return ent;
}

	
	
void world::Entity::WorldUpdate( const float x, const float y )
{
	const unsigned short cell_x = CellFromFloat( x );
	const unsigned short cell_y = CellFromFloat( y );
	
	if( cell_x != this->cell_x || cell_y != this->cell_y )
	{
		const unsigned int hash = CellHash( cell_x, cell_y );
		world::Entity *list = cells[ hash ];
		
		if( this->cell_x | this->cell_y ) 
			this->WorldDelete();

		this->cell_x = cell_x;
		this->cell_y = cell_y;

		if( list )
		{
			this->next = list->next;
			list->next = this;
		}
		else
		{
			this->next = NULL;
			cells[ hash ] = this;
		}
		
		assert( entities++ >= 0 );
	}
}


void world::Entity::WorldDelete( void )
{
	const unsigned int hash = CellHash( this->cell_x, this->cell_y );
	world::Entity *list = cells[ hash ];
	
	assert( list );
	
	if( list == this )
	{
		if( list->next )
		{
			cells[ hash ] = list->next;
		}
		else
		{
			cells.erase( hash );
		}
	}
	else
	{
		assert( list->next );
		while( list->next != this ) {
			list = list->next;
			assert( list );
		}
			
		assert( list && list->next == this );

		list->next = this->next;
	}
			
	assert( --entities >= 0 );

	this->cell_x = 0;
	this->cell_y = 0;
	this->next = NULL;
}



world::NearbyIterator::NearbyIterator( const float pos_x, const float pos_y, const float radius )
{
	this->cell_min_x = CellFromFloat( pos_x - radius );
	this->cell_min_y = CellFromFloat( pos_y - radius );
	this->cell_max_x = CellFromFloat( pos_x + radius );
	this->cell_max_y = CellFromFloat( pos_y + radius );

	this->Reset();
}

		
world::NearbyIterator::NearbyIterator( const float box_min_x, const float box_min_y, const float box_max_x, const float box_max_y )
{
	this->cell_min_x = CellFromFloat( box_min_x );
	this->cell_min_y = CellFromFloat( box_min_y );
	this->cell_max_x = CellFromFloat( box_max_x );
	this->cell_max_y = CellFromFloat( box_max_y );
	
	this->Reset();
}


void world::NearbyIterator::Reset( void )
{
	this->cell_cur_x = this->cell_min_x - 1;
	this->cell_cur_y = this->cell_min_y;

	this->ent = NULL;
}


world::Entity * world::NearbyIterator::Next( void )
{
	if( this->ent ) this->ent = this->ent->next;
	
	while( !this->ent )
	{
		if( this->cell_cur_x < this->cell_max_x )
		{
			this->cell_cur_x++;
		}
		else if( this->cell_cur_y < this->cell_max_y )
		{
			this->cell_cur_x = this->cell_min_x;
			this->cell_cur_y++;
		}
		else
		{
			break;
		}
		
		CellMap::iterator it = cells.find( CellHash( this->cell_cur_x, this->cell_cur_y ) );
		this->ent = ( it == cells.end() ? NULL : it->second );
	}

	return this->ent;
}


/* //####TODO: NOT TESTED
world::Entity * world::NearbyRayIterator::Next( void )
{
	if( this->ent ) this->ent = this->ent->next;
	
	while( !this->ent && this->count0 <= this->total )
	{
		const float x = this->px + this->count0 * this->mx;
		const float y = this->py + this->count0 * this->my;
		
		const int cx = CellFromFloat( x ) + ( this->mx == 1.0f ? 0 : this->count1 );
		const int cy = CellFromFloat( y ) + ( this->mx == 1.0f ? this->count1 : 0 );
		
		const float rx = x - CellToFloat( cx );
		const float ry = y - CellToFloat( cy );
		const float rr = this->radius + world::CELL_SIZE;
		if( rx*rx < rr*rr && ry*ry < rr*rr ) {
			CellMap::iterator it = cells.find( CellHash( cx, cy ) );
			this->ent = ( it == cells.end() ? NULL : it->second );
		}
		
		if( this->count1++ >= 1 ) {
			this->count1 = -1;
			this->count0++;
		}
	}

	return this->ent;
}
*/

