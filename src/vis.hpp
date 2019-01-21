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


#ifndef __VIS_HPP__
#define __VIS_HPP__


/// Módulo encargado de gestionar la visualización de la simulación.
namespace vis // visualization
{
	
	/// Parámetros de configuración de la visualización.
	struct Config {
		Config() : cfg_resources(0), cfg_plugins(0), veh_mesh(0), ped_mesh(0), obj_mesh(0), vsync(0) { }
		const char *cfg_resources;	///< Ruta del fichero de configuración de recursos de Ogre (\a resources.cfg).
		const char *cfg_plugins;	///< Ruta del fichero de configuración de plugins de Ogre (\a plugins.cfg).
		const char *veh_mesh;		///< Nombre de la malla para los vehículos. La malla debe existir en el fichero \a .scene. La malla tiene que estar normalizada (ancho=2,largo=2,alto=1).
		const char *ped_mesh;		///< Nombre de la malla para los peatones. La malla debe existir en el fichero \a .scene. La malla tiene que estar normalizada (radio=1,alto=1).
		const char *obj_mesh;		///< Nombre de la malla para los objetos. La malla debe existir en el fichero \a .scene. La malla tiene que estar normalizada (radio=1,alto=1).
		bool		vsync;			///< Indica si realizar o no la sincronización vertical con el refresco del monitor.
	};
	
	/// Reserva e inicializa recursos.
	void Initialize( const Config &config );

	/// Libera recursos.
	void Finalize( void );

	/// Actualiza el estado visual de las entidades de la simulación.
	/// Esta función debe ser llamada una vez por frame.
	/// \param [in] dt  Tiempto transcurrido desde el último frame.
	/// \return         Indica si la visualización continua o hay que finalizar.
	bool Update( const float dt );

} // namespace vis
	

#endif // __VIS_HPP__