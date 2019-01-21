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


#ifndef __WORLD_HPP__
#define __WORLD_HPP__


/// Módulo encargado de gestionar las entidades dinámicas del mundo.
///
/// El mundo se divide en celdas regulares (cuadrícula) de tamaño world::CELL_SIZE.
/// Las entidades se insertan/extraen automaticamente en estas celdas según su posición, de modo que podemos recoger todas las entidades entorno a un punto o AABB.
/// \note El tamaño de celda world::CELL_SIZE debe ser ajustado en función del tamaño/velocidad/densidad de las entidades para obetener un buen rendimiento.
///
/// Ejemplo de uso:
/// \code{.cpp}
/// 	class AnimalDog;
/// 	class AnimalCat;
/// 	class AnimalMouse;
/// 
/// 	class Animal : public world::Entity {
/// 		public:
/// 			enum { DOG=101, CAT, MOUSE };
/// 		public:
/// 			Animal( int type ) : Entity(type) { ... }
/// 			Update( float dt );
/// 			float pos_x, pos_y;
/// 			...
/// 		public:
/// 			virtual OnDog( dt, AnimalDog *dog );
/// 			virtual OnCat( dt, AnimalCat *cat );
/// 			virtual OnMouse( dt, AnimalMouse *mouse );
/// 			virtual OnUpdate( dt );
/// 			...
/// 	}
/// 
/// 	class AnimalDog : public Animal {
/// 		public:
/// 			AnimalDog() : Animal(Animal::DOG) { ... }
/// 			... virtual OnDog / OnCat / OnMouse / OnUpdate ...
/// 	};
/// 
/// 	class AnimalCat : public Animal {
/// 		public:
/// 			AnimalCat() : Animal(Animal::CAT) { ... }
/// 			... virtual OnDog / OnCat / OnMouse / OnUpdate ...
/// 	};
/// 
/// 	class AnimalMouse : public Animal {
/// 		public:
/// 			AnimalMouse() : Animal(Animal::MOUSE) { ... }
/// 			... virtual OnDog / OnCat / OnMouse / OnUpdate ...
/// 	};
/// 
/// 
/// 	Animal::Update( float dt )
/// 	{
/// 		// todas las entidades unos 4 metros alrededor
/// 		world::NearbyIterator nearby( this->pos_x, this->pos_y, 4.0f );
/// 		while( true )
/// 		{
/// 			world::Entity *ent = nearby.Next();
/// 			if( !ent ) break;
/// 			
/// 			if( ent == &this ) continue;
/// 			
/// 			switch( ent->GetType() )
/// 			{
/// 				case Animal::DOG:  // un perro cercano
/// 					this->OnDog( dt, (AnimalDog*)ent );
/// 				break;
/// 				
/// 				case Animal::CAT:  // un gato cercano
/// 					this->OnCat( dt, (AnimalCat*)ent );
/// 				break;
/// 				
/// 				case Animal::MOUSE:  // un ratón cercano
/// 					this->OnMouse( dt, (AnimalMouse*)ent );
/// 				break;
/// 			}
/// 		}
/// 		
/// 		this->OnUpdate( dt );
/// 		
/// 		// notificar un cambio de posición en la entidad
/// 		this->WorldUpdate( this->pos_x, this->pos_y );
/// 	}
/// 
/// 
/// 	static std::vector<AnimalDog>   dogs(  5 );
/// 	static std::vector<AnimalCat>   cats( 10 );
/// 	static std::vector<AnimalMouse> mice( 20 );
/// 
/// 	void UpdateAnimals( float dt )
/// 	{
/// 		for( int i = 0; i < dogs.size(); i++ )
/// 			dogs[i].Update( dt );
/// 			
/// 		for( int i = 0; i < cats.size(); i++ )
/// 			cats[i].Update( dt );
/// 			
/// 		for( int i = 0; i < mice.size(); i++ )
/// 			mice[i].Update( dt );
/// 	}
/// \endcode
///
namespace world
{

	
	static const float CELL_SIZE     = 8.0f;				///< Tamaño de las celdas del mundo.
	static const float CELL_SIZE_INV = 1.0f / CELL_SIZE;	///< Inversa del tamaño de las celdas del mundo.
	
	/// Clase base de las entidades dinámicas del mundo.
	/// Cada tipo de entidad tiene un identificador \a type que podemos utilizar para hacer \a downcasting. \n
	/// Cada vez que la entidad cambia su posición hay que notificarlo mediante Entity::WorldUpdate(). \n
	/// Ver ::world para un ejemplo de uso. \n
	class Entity
	{
		public:

			/// Constructor de la entidad base.
			/// Las entidades no son insertadas en el mundo hasta que no se llame a Entity::WorldUpdate().
			/// \param [in] type  Tipo de entidad que nos permite realizar \a downcasting.
			Entity( unsigned int type ) : type(type), cell_x(0), cell_y(0), next(0) { }
			
			/// Destructor de la entidad base.
			/// Al destruirse una entidad se extrae automaticamente del mundo.
			virtual ~Entity() { if( cell_x | cell_y ) WorldDelete(); }
			
			/// Obtiene el tipo de la entidad base.
			/// \return  Tipo de entidad.
			inline unsigned int GetType( void ) const { return this->type; }

			/// Notificar que la entidad ha cambiado de posición.
			/// Esta función hay que llamarla cada vez que la entidad cambia de posición para mantener las celdas actualizadas.
			/// \param [in] x  Posición X actual de la entidad.
			/// \param [in] y  Posición Y actual de la entidad.
			void WorldUpdate( const float x, const float y );
			
			/// Extrae la entidad del mundo.
			/// Despues de llamar a Entity::WorldDelete() el mundo deja de tener referencias a la entidad, \n
			/// por tanto no se encontrará mediante world::NearbyIterator. \n
			/// Es completamente valido volver a insertar la entidad mediante Entity::WorldUpdate(). \n
			void WorldDelete( void );

			/// Indica si la entidad está insertada en el mundo.
			inline bool WorldInserted( void ) const { return (bool) ( cell_x | cell_y ); }
			
		private:
		//public:

			friend  bool world::Initialize( void );
			friend  void world::Finalize( void );
			friend  void world::QueuePushBack( world::Entity *ent );
			friend  world::Entity * world::QueuePopFront( void );
			friend  class NearbyIterator;
	
			const unsigned int  type;
			unsigned short 		cell_x;
			unsigned short 		cell_y;
			Entity	       		*next;
	};
	
	
	/// Nos permite iterar sobre las entidades del mundo alrededor de un punto o AABB.
	/// Ver ::world para un ejemplo de uso.
	class NearbyIterator
	{
		public:		

			/// Constructor del iterador a partir de un punto y un radio.
			/// Este constructor es equivalente a NearbyIterator( pos_x-radius, pos_y-radius, pos_x+radius, pos_y+radius ).
			/// \param [in] pos_x   Coordenada X de la posición/centro.
			/// \param [in] pos_y   Coordenada Y de la posición/centro.
			/// \param [in] radius  Radio de búsqueda.
			NearbyIterator( const float pos_x, const float pos_y, const float radius );

			/// Constructor del iterador a partir de un AABB.
			/// \param [in] box_min_x   Coordenada X del punto inferior izquierda.
			/// \param [in] box_min_y   Coordenada Y del punto inferior izquierda.
			/// \param [in] box_max_x   Coordenada X del punto superior derecha.
			/// \param [in] box_max_y   Coordenada Y del punto superior derecha.
			NearbyIterator( const float box_min_x, const float box_min_y, const float box_max_x, const float box_max_y );
			
			/// Reinicia el iterador.
			void Reset( void );
				
			/// Devuelve la siguiente entidad cercana o NULL para finalizar.
			/// El orden de las entidades no está definido. \n
			/// El radio o alcance de la búsqueda puede ser mayor al indicado en el constructor pero nunca menor. \n
			/// \return  Siguiente entidad encontrada o NULL.
			/// \warning Nunca eliminar entidades o realizar llamadas a Entity::WorldUpdate() o Entity::WorldDelete()
			///          mientras se está iterando sobre ellas.
			Entity * Next( void );
		
		private:
		
			unsigned short cell_min_x, cell_min_y;
			unsigned short cell_max_x, cell_max_y;
			unsigned short cell_cur_x, cell_cur_y;
			Entity *ent;
	};

/* //####TODO: NOT TESTED
	/// Nos permite iterar sobre las entidades del mundo entorno a una recta y un radio.
	/// Ver ::world para un ejemplo de uso.
	class NearbyRayIterator
	{
		public:		

			/// Constructor del iterador a partir de un rayo, radio y distancia.
			/// \param [in] px			Coordenada X del origen del rayo.
			/// \param [in] py			Coordenada Y del origen del rayo.
			/// \param [in] dx			Coordenada X de la dirección del rayo.
			/// \param [in] dy			Coordenada Y de la dirección del rayo.
			/// \param [in] radius		Radio/anchura del rayo.
			/// \param [in] distance	Distancia/longitud del rayo.
			NearbyRayIterator( const float px, const float py, const float dx, const float dy, const float radius, const float distance )
				: px(px), py(py), radius(radius), count0(0), count1(-1), ent(0),
				  mx   ( ( dx >= dy ? 1.00f : dx/dy ) * world::CELL_SIZE ),
				  my   ( ( dx >= dy ? dy/dx : 1.00f ) * world::CELL_SIZE ),
				  total( ( dx >= dy ? dx : dy ) * (distance+radius) * world::CELL_SIZE_INV )
				{ }
				
			/// Reinicia el iterador.
			void Reset( void ) {
				this->count0 =  0;
				this->count1 = -1;
				this->ent = NULL;
			}

			/// Devuelve la siguiente entidad cercana o NULL para finalizar.
			/// El orden de las entidades no está definido. \n
			/// El radio o alcance de la búsqueda puede ser mayor al indicado en el constructor pero nunca menor. \n
			/// \return  Siguiente entidad encontrada o NULL.
			/// \warning Nunca eliminar entidades o realizar llamadas a Entity::WorldUpdate() o Entity::WorldDelete()
			///          mientras se está iterando sobre ellas.
			Entity * Next( void );
		
		private:

			const float px, py;
			const float radius;
			int         count0;
			int         count1;
			Entity      *ent;
			const float mx, my;
			const int   total;
	};
*/

	/// Reserva e inicializa recursos.
	bool Initialize( void );

	/// Libera recursos.
	void Finalize( void );


	/// Añade una entidad a la cola temporal de entidades.
	/// Las entidades en la cola temporal son extraidas del mundo, no es válido realizar llamadas a world::WorldUpdate y world::WorldDelete.
	/// \pram ent   Entidad a añadir a la pila.
	void QueuePushBack( world::Entity *ent );
	
	/// Extrae la primera entidad de la cola temporal.
	/// \return   Entidad extraida de la cola.
	Entity * QueuePopFront( void );
}



#endif // __WORLD_HPP__
