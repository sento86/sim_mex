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


#ifndef __NAV_HPP__
#define __NAV_HPP__


/// Módulo encargado de gestionar la navegación de los vehículos y peatones.
namespace nav // navigation
{

	/// Submódulo para gestionar la navegación de los vehículos.
	namespace veh // vehicle
	{
		
		/// Tipos de señales.
		/// \warning Estos valores deben ser iguales a los del script python export_veh_graph.py.
		namespace sign {
			enum Signal { NONE=0, SPAWN, YIELD, STOP, SEMAPHORE, SPEED };
		}
		
		/// Tipos de ruta.
		/// Cada nodo puede tener hasta dos nodos sucesores. El tipo de ruta indica que camino debe tomar el vehículo.
		namespace route {
			enum { NONE=0, LEFT, RIGHT, ANY };
		}

		/// Nodo del grafo dirigido de navegación de los vehículos.
		/// Los nodos contienen información extra para facilitar y acelerar la toma de decisiones de los vehículos. \n
		/// Los nodos solamente pueden tener hasta dos sucesores y hasta dos predecesores, por lo que a un nodo se puede acceder por la \a izquierda=0 o por la \a derecha=1,
		/// permitiéndonos distinguir señales y enrutamiento distinto por cada acceso Node::from.
		struct Node {
			struct {
				unsigned char sign  : 4;	///< Señal por este acceso.
				unsigned char route : 4;	///< Enrutamiento por este acceso.
			} from[2];						///< Acceso por la \a izquierda=0 o por la \a derecha=1.
			unsigned char semaphore;		///< Índice de tipo de semáforo o 0 si no es un nodo semáforo.
			unsigned char margin;			///< Porcentaje de margen de seguridad en incorporaciones.
			unsigned int  prev[2];			///< Índice de los dos nodos anteriores (izquierdo y derecho).
			unsigned int  next[2];			///< Índice de los dos nodos siguientes (izquierdo y derecho).
			float         x, y, z;			///< Posición del nodo.
		};

		struct PlanNode;	// private implementation

		/// Grafo dirigido de navegación de los vehículos.
		struct Graph {
			unsigned int num_nodes;		///< Número total de nodos en el grafo. \warning El primer nodo de índice 0 no es válido.
			unsigned int num_spawns;	///< Número de nodos de nacimiento.     \warning El primer nodo de índice 0 no es válido.
			Node 		 *nodes;		///< Array de nodos del grafo.
			PlanNode	 (*pnodes)[2];	///< Array de nodos de planificación.
		};

		/// Carga el fichero con los datos del grafo de navegación de vehículos.
		/// \param  [in] file  Nombre del fichero con los datos del grafo.
		/// \return            Puntero al grafo cargado. NULL si hay error.
		const Graph * Load( const char *file );
		
		/// Libera la memoria reservada por el grafo de navegación de vehículos.
		/// \param [in,out] graph  Puntero al grafo a liberar.
		void Free( const Graph *&graph );
		
		/// Devuelve las coordenadas del nodo de nacimiento indicado.
		/// \param [in] graph        Grafo de navegación de vehículos.
		/// \param [in] index_spawn  Índice del nodo de nacimiento.
		/// \return                  Puntero al nodo de nacimiento.
		const nav::veh::Node * GetRespawnNode( const nav::veh::Graph *graph, const int index_spawn );

		/// Planificación de los vehículos.
		/// Todos los vehículos deben heredar de esta clase para ser guiados sobre el grafo de navegación. \n
		/// También permite a los vehículos obtener información sobre preferencias, señales y posibles colisiones con otros vehículos. \n
		/// En esta implementación, el comportamiento ante bifurcaciones es aleatorio.
		class Plan
		{
			public:

				/// Constructor del planificador de vehículos.
				Plan() : graph(0), bits(0), prev(0), curr(0), speed_limit_kmh(0) { }
				
				/// Destructor del planificador de vehículos.
				virtual ~Plan() { }
			
				/// Inicializa el planificador sobre un nodo de nacimiento.
				/// \param [in] graph        Grafo de navegación sobre el que planificar.
				/// \param [in] speed        Velocidad de inicio del vehículo. (metros/segundo)
				/// \param [in] index_spawn  Índice del nodo de nacimiento.
				/// \return                  Puntero al nodo de nacimiento.
				const nav::veh::Node * Respawn( const nav::veh::Graph *graph, const float speed, const int index_spawn );
				
				/// Inicializa el planificador sobre un nodo de nacimiento.
				/// Reutiliza el graph asignado anteriormente.
				/// \param [in] speed        Velocidad de inicio del vehículo. (metros/segundo)
				/// \param [in] index_spawn  Índice del nodo de nacimiento. Si el índice es -1 se utiliza el número aleatorio \a bits (ver veh::Plan::SetTurnBitsRandom()).
				/// \return                  Puntero al nodo de nacimiento.
				inline const nav::veh::Node * Respawn( const float speed, const int index_spawn=-1 ) {
					return this->Respawn( this->graph, speed, index_spawn );
				}

			public:
			
				/// Información sobre posibles colisiones
				struct Info {
					const Node	  *node;		///< Nodo en el que sucede la colisión o NULL si no hay ninguna.
					Plan		  *plan;		///< El otro plan con el que colisiona o NULL si no es un plan.
					float		  dist;			///< Distancia a la que sucederá la colisión. (metros)
					float		  time;			///< Tiempo en el que sucederá la colisión. (segundos)
					float		  curvature;	///< Parámetro que indica la curvatura de la ruta. Entre 0 y 1 ( 0=muchas_curvas, 1=linea_recta ).
					float		  speed_limit;	///< Límite máximo de velocidad para este tramo de la ruta. (metros/segundo)
					unsigned char myway;		///< Indica si se trata del propio camino del vehículo: no=0, si=1.
					unsigned char semaphore;	///< Índice del semáforo o 0 si el nodo no es un semáforo.
				};
				
				/// Realiza la planificación de un vehículo.
				/// Realiza una búsqueda sobre el grafo de navegación obteniendo el nodo al que tiene que dirigirse el vehículo.
				/// \param [in]  x          Posición X actual del vehículo.
				/// \param [in]  y          Posición Y actual del vehículo.
				/// \param [in]  length     Longitud del vehículo.
				/// \param [in]  speed      Velocidad actual del vehículo.
				/// \param [in]  time       Tiempo de planificación.
				/// \param [out] collision  Información sobre posibles colisiones.
				/// \return                 Nodo del grafo de navegación al que tiene que dirigirse el vehículo.
				const nav::veh::Node * Planify( const float x, const float y, const float length, const float speed, const float time, Info *const info );
				
				/// Puntero a función para acceder a los planes de alrededor.				
				/// Ver veh::Plan::Nearby().
				/// \param [in,out] plan   El plan que realiza la búsqueda.
				/// \param [in,out] other  Un plan cercano.
				typedef void (*NearbyCallback) ( veh::Plan *plan, veh::Plan *other );
				
				/// Permite recoger los planes cercanos a la ruta del plan mediante una función callback.
				/// \param [in] dist      Distancia desde la ruta del plan.
				/// \param [in] callback  Puntero a función para recoger los planes cercanos.
				void Nearby( const float dist, NearbyCallback callback );
				
			public:
			
				/// Dirección de giro ante un bifurcación.
				/// Solamente existen dos posiblidades de giro ya que los nodos solamente disponen de dos nodos sucesores.
				enum TurnDirection { TURN_LEFT=0, TURN_RIGHT=1 };

				/// Recoge los 32 siguientes giros codificados en bits.
				/// \return Los 32 siguientes giros codificados en 32 bits.
				inline unsigned int GetTurnBits( void ) const {
					return this->bits;
				}

				/// Obtiene la dirección de un futuro giro.
				/// \param [in] index  Indice del bit o número de giro.
				/// \return            Dirección del futuro giro.
				inline TurnDirection GetTurnDirection( const int index ) const {
					return (TurnDirection) ( ( this->bits >> index ) & 1 );
				}
				
				/// Inicializa aleatoriamente los bits de giro.
				/// \param [in] seed  Semilla para inicializar los bits de giro.
				inline void SetTurnBitsRandom( const unsigned int seed ) {
					this->bits = seed * 3941169319u ^ 2902958803u;						// next random number
				}			
				
				/// Obtiene la próxima dirección de giro y avanza al siguiente.
				/// \return  Dirección del próximo giro.
				inline TurnDirection Turn( void ) {
					TurnDirection turn = (TurnDirection) ( this->bits & 1 );			// first turn bit --> left/right
					const unsigned int r = this->bits * 3941169319u ^ 2902958803u;		// next random number
					this->bits = ( this->bits >> 1 ) | ( r & 0x80000000u );				// insert a new bit
					return turn;
				}


			public:
			//protected:
			
				const nav::veh::Graph   *graph;
				unsigned int			bits;
				unsigned int			prev;
				unsigned int			curr;
				unsigned char			speed_limit_kmh;
		};

	} // namespace veh
	

	/// Submódulo para gestionar la navegación de los peatones.
	namespace ped // pedestrian
	{
		
		/// Tipo de señal.
		/// \warning Estos valores deben ser iguales a los del script python export_ped_graph.py.
		namespace sign {
			enum { NONE=0, SPAWN, SEMAPHORE };
		}

		/// Nodo del grafo de navegación de peatones.
		/// Se trata de un grafo no dirigido con nodos que pueden tener hasta 4 nodos adyacentes. \n
		/// Los nodos contienen información extra para facilitar y acelerar la toma de decisiones de los peatones. \n
		struct Node {
			unsigned char sign;				///< Tipo de señal en el nodo.
			unsigned char semaphore;		///< Índice de tipo de semáforo o 0 si no existe ninguno.
			unsigned char _pad;
			unsigned char count;			///< Número de nodos adyacentes, entre 1 y 4.
			struct {
				unsigned int ang  :  8;		///< Ángulo de la dirección del nodo adyacente ( 0=256=2PI, 128=PI, 64=PI/2 ).
				unsigned int next : 24;		///< Índice del nodo adyacente.
			} na[4];						///< Los ángulos e índices de los cuatro nodos adyacentes.
			float x, y, z;					///< Posición del nodo.
		};

		/// Grafo dirigido de navegación de los peatones.
		struct Graph {
			unsigned int num_nodes;			///< Número total de nodos.      \warning El primer nodo de índice 0 no es válido.
			unsigned int num_spawns;		///< Número nodos de nacimiento. \warning El primer nodo de índice 0 no es válido.
			Node 		 *nodes;			///< Array de nodos del grafo de navegación.
		};

		/// Carga el fichero con los datos del grafo de navegación de peatones.
		/// \param  [in] file  Nombre del fichero con los datos del grafo.
		/// \return            Puntero al grafo cargado. NULL si hay error.
		const Graph * Load( const char *file );
		
		/// Libera la memoria reservada por el grafo de navegación de peatones.
		/// \param [in,out] graph  Puntero al grafo a liberar.
		void Free( const Graph *&graph );

		/// Devuelve las coordenadas del nodo de nacimiento indicado.
		/// \param [in] graph        Grafo de navegación de peatones.
		/// \param [in] index_spawn  Índice del nodo de nacimiento.
		/// \return                  Puntero al nodo de nacimiento.
		const nav::ped::Node * GetRespawnNode( const nav::ped::Graph *graph, const int index_spawn );

		/// Planificación de los peatones.
		/// Todos los peatoenes deben heredar de esta clase para ser guiados sobre el grafo de navegación. \n
		/// En esta implementación, el comportamiento ante bifurcaciones es aleatorio.
		class Plan
		{
			public:

				/// Constructor del planificador de peatones.
				Plan() : graph(0), bits(0), curr(0) { }			

				/// Destructor del planificador de peatones.
				virtual ~Plan() { }
			
				/// Inicializa el planificador sobre un nodo de nacimiento.
				/// \param [in] graph        Grafo de navegación sobre la que planificar.
				/// \param [in] index_spawn  Índice del nodo de nacimiento.
				/// \return                  Puntero al nodo de nacimiento.
				const nav::ped::Node * Respawn( const nav::ped::Graph *graph, const int index_spawn );
				
				/// Inicializa el planificador sobre un nodo de nacimiento.
				/// Reutiliza el graph asignado anteriormente.
				/// \param [in] index_spawn  Índice del nodo de nacimiento.
				/// \return                  Puntero al nodo de nacimiento.
				inline const nav::ped::Node * Respawn( const int index_spawn ) {
					return this->Respawn( this->graph, index_spawn );
				}

			public:
			
				/// Realiza la planificación de un peatón.
				/// Realiza una búsqueda sobre el grafo de navegación obteniendo el nodo al que tiene que dirigirse el peatón.
				/// \param [in] x         Posición X actual del peatón.
				/// \param [in] y         Posición Y actual del peatón.
				/// \param [in] ang       Ángulo actual del peatón (radianes).
				/// \param [in] distance  Distancia (lookahead) mínima para pasar al siguiente nodo.
				/// \return               Puntero al nodo al que tiene que dirigirse el peatón.
				const nav::ped::Node * Planify( const float x, const float y, const float ang, const float distance );
				
				/// Cambia la ruta del peatón sobre el grafo de navegación.
				/// Útil cuando un peaton intenta alcanzar un nodo inalcanzable.
				/// \param [in] ang       Ángulo actual del peatón (radianes).
				/// \return               Puntero al nodo al que tiene que dirigirse el peatón.
				const nav::ped::Node * RePlanify( const float ang );
				
			public:
			
				/// Inicializa aleatoriamente los bits de giro.
				/// \param [in] seed  Semilla para inicializar los bits de giro.
				inline void SetRandomSeed( const unsigned int seed ) {
					this->bits = seed * 3941169319u ^ 2902958803u;				// next random number
				}			
				
				/// Obtiene los bits de giro y avanza a los siguientes.
				/// \return  Dirección de los giros codificados en 32 bits.
				inline unsigned int GetRandom( void ) const {
					this->bits = (this->bits>>2) * 3941169319u ^ 2902958803u;	// next random number
					return this->bits;											// curr = node.next[ two bits ]
				}
				
			//protected:
			public:
			
				const nav::ped::Graph   *graph;
				mutable unsigned int	bits;
				unsigned int			curr;
				unsigned int			prev;
		};

	} // namespace ped


	/// Submódulo para gestionar el estado de los semáforos.
	/// Todos los semáforos están sincronizados en tiempo por un único contador de segundos, actualizado mediante nav::Update(). \n
	/// No se trata de semáforos en sí, sino de tipos de semáforos, es decir, podemos tener varios semáforos iguales controlados por un único tipo de semáforo.
	namespace sem // semaphore
	{

		/// Número máximo de tipos de semáforos.
		static const int MAX = 256;
		
		/// Configura los tiempos de un tipo de semáforo.
		/// \param [in] idx         Índice del tipo de semáforo. Mayor que 0 y menor que sem::MAX.
		/// \param [in] secs_total  Tiempo total de ciclo o período.
		/// \param [in] secs_green  Tiempo de duración en estado verde.
		/// \param [in] secs_phase  Tiempo de fase para alternar estados.
		void SetTimes( const int idx, int secs_total, int secs_green, int secs_phase );
		
		/// Permite conocer si un semáforo está en verde.
		/// \param [in] idx  Índice del tipo de semáforo.
		/// \return          Verdadero o falso si se encuentra en estado verde o no.
		bool IsGreen( const int idx );
		
		/// Carga el fichero de configuracion de tipos de semáforos.
		/// Cada linea del fichero contiene la descripción de un tipo de semáforo: IDX TOTAL GREEN PHASE. Ver sem::SetTimes para la descripción de cada parámetro.
		/// \param [in] file  Fichero de configuración de los tipos semáforos.
		/// \return           Verdadero o falso si se ha cargado correctamente o no.
		bool Load( const char *file );

	} // namespace sem


	/// Inicializa los tres submódulos.	
	void Initialize( void );
	
	/// Finaliza los tres submódulos.
	void Finalize( void );

	/// Actualiza los tres submódulos.
	/// Esta función debe ser llamada una vez por frame.
	/// \param [in] dt  Tiempo transcurrido desde la última llamada (en segundos).
	void Update( const float dt );

} // namespace nav
	

#endif // __NAV_HPP__
