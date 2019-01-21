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


#ifndef __SIM_HPP__
#define __SIM_HPP__


#include "world.hpp"
#include "phys.hpp"
//#include "nav.hpp"


//#include <safetrans_msgs/event.h>


/// Módulo encargado de gestionar la lógica de la simulación.
namespace sim // simulator
{	
	
	/// Tipo de entidades en el simulador.
	namespace ent {
		enum Type { NONE=0, BUS, VEHICLE, PEDESTRIAN, OBJECT, _SIZE };
	}

	static const float VEH_WIDTH_MIN			=  1.8f;								///< Anchura mínima de los vehículos.
	static const float VEH_WIDTH_MAX			=  2.2f;								///< Anchura máxima de los vehículos.
	static const float VEH_LENGTH_MIN			=  4.5f;								///< Longitud mínima de los vehículos.
	static const float VEH_LENGTH_MAX			=  4.8f;								///< Longitud máxima de los vehículos.
	static const float VEH_ENVIRONMENT_RADIUS	= 20.0f;								///< Distancia máxima de influencia de los vehículos.
	static const float VEH_SPEED_RESPAWN		= 40.0f * (1000.0f/60/60);				///< Máxima velicidad de los vehículos: 40 Km/h.
	static const float VEH_ACCEL_MAX			= 2.0f;									///< Aceleración máxima de los vehículos (m/s^2).
	static const float VEH_ACCEL_MIN			= -1.1f * VEH_ACCEL_MAX;				///< Aceleración mínima de los vehículos.
	static const float VEH_ECCENTICITY_MIN		= 0.5f;									///< Excentricidad mínima de la elipse: 0.00f=circle, 0.99f=line.
	static const float VEH_ECCENTICITY_MAX		= 0.8f;									///< Excentricidad máxima de la elipse: 0.00f=circle, 0.99f=line.
	static const float VEH_STEER_ANGLE_MAX		= 0.4f*3.141592f;						///< Máximo ángulo de giro de las ruedas delanteras de los vehículos.

	static const float PED_HEIGHT_MIN			= 1.50f;	///< Altura mínima de los peatones.
	static const float PED_HEIGHT_MAX			= 1.90f;	///< Altura máxima de los peatones.
	static const float PED_RADIUS_MIN			= 0.30f;	///< Radio mínimo de los peatones.
	static const float PED_RADIUS_MAX			= 0.40f;	///< Radio máximo de los peatones.
	static const float PED_SPEED_MIN			= 0.75f;	///< Velocidad mínimo de los peatones.
	static const float PED_SPEED_MAX			= 1.25f;	///< Velocidad máxima de los peatones.
	static const float PED_WSPEED_MAX			= 2.00f;	///< Velocidad angular de los peatones.
	static const float PED_SPEED_MULT_GREEN		= 1.25f;	///< Escala de velocidad cuando el peatón se encuentra en un semáforo en verde.
	static const float PED_SPEED_MULT_RED		= 1.75f;	///< Escala de velocidad cuando el peatón se encuentra en un semáforo en rojo.
	static const float PED_FALL_SPEED			= -2.0f;	///< Velocidad de caída en m/s. No tenemos en cuenta la gravedad ni la aceleración.

	class Bus : public phys::UserVehicleID, public world::Entity  //####BUS
	{
		public:
			Bus() : Entity(ent::BUS) { }
		public:
			float width, length, height, speed;
			float px, py, pz, _pad0;
			float dx, dy, _pad1, _pad2;
			short orientation[4];
	};

	/// Parámetros de configuración del simulador.
	struct Config {
		Config() : collision_mesh(0) { }
		const char *collision_mesh;		///< Ruta de la malla de colisión de la escena. Formato OBJ: solo vértices y triángulos, sin uv ni normales, XYZ=(right,forward,up).
	};
	
	/// Reserva e inicializa recursos.
	void Initialize( const Config &config );
	
	/// Libera recursos.
	void Finalize( void );

	/// Actualiza el estado de las entidades de la simulación.
	/// Esta función debe ser llamada una vez por frame.
	/// \param [in] dt  Tiempo transcurrido desde la última llamada (en segundos).
	/// \return         Indica si la simulación continua o hay que finalizar.
	//bool Update( const float dt );
	bool Update( const float dt );

	const Bus * GetBus( void );	//####BUS


} // namespace sim


//#define VEHICLES				100
//#define PEDESTRIANS				150
//#define PEDESTRIANS_DUMP 		20	//####PEDESTRIAN DUMP (DUMP=CROSSWALK+BUSSTOP+CORNER)
//#define PEDESTRIANS_CROSSWALK 	6
//#define PEDESTRIANS_BUSSTOP 	6
//#define PEDESTRIANS_CORNER 		8
///*#define PEDESTRIANS_DUMP 		10	//####PEDESTRIAN DUMP (DUMP=CROSSWALK+BUSSTOP+CORNER)
//#define PEDESTRIANS_CROSSWALK 	0
//#define PEDESTRIANS_BUSSTOP 	0
//#define PEDESTRIANS_CORNER 		0*/
//#define OBJECTS 				5
//#define VEHICLES_RANDOM			1
//#define PEDESTRIANS_RANDOM		1

#define VEHICLES				0
#define PEDESTRIANS				0
#define PEDESTRIANS_DUMP 		0	//####PEDESTRIAN DUMP (DUMP=CROSSWALK+BUSSTOP+CORNER)
#define PEDESTRIANS_CROSSWALK 	0
#define PEDESTRIANS_BUSSTOP 	0
#define PEDESTRIANS_CORNER 		0
#define OBJECTS 				0
#define VEHICLES_RANDOM			0
#define PEDESTRIANS_RANDOM		0

	

#endif // __SIM_HPP__
