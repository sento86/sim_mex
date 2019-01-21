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


#include <assert.h>

#include <vector>

#include <OgreManualObject.h>
#include "BaseApplication.h"

#include "sim.hpp"
#include "vis.hpp"



namespace vis
{

	/// Información visual de la entidad vehículo
	struct Vehicle {
		Vehicle() : node(0), ellipse(0), collision(0) { }
		Ogre::SceneNode *node;
		Ogre::SceneNode *ellipse;
		Ogre::SceneNode *collision;
	};

	/// Información visual de la entidad peatón
	struct Pedestrian {
		Pedestrian() : node(0) { }
		Ogre::SceneNode *node;
	};

	struct Bus {	//####BUS
		Bus() : node(0) { }
		Ogre::SceneNode *node;
	};
}


static std::vector< vis::Vehicle    > vehicles;		///< Array con información visual de los vehículos. Los índices coinciden con el array devuelto por sim::GetVehicles().
static std::vector< vis::Pedestrian > pedestrians;	///< Array con información visual de los peatones. Los índices coinciden con el array devuelto por sim::GetPedestrians().
static vis::Bus	bus;	//####BUS



/// Genera la represenciación visual del grafo de navegación de vehículos.
/// Cada nodo es pintado de un color diferente, en función de la señal de tráfico. \n
/// Al tratarse de un grafo dirigido, el color de las aristas indica el sentido de la dirección.
static void BuildVehicleGraph( const vis::Config &config, Ogre::SceneManager *scene_mgr )
{
	const nav::veh::Graph *nvg = sim::GetNavigationVehicleGraph();
	
	Ogre::ManualObject *graph = scene_mgr->createManualObject( "veh_graph_" );
	graph->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
		
	Ogre::ColourValue colour_gray   ( 0.2f, 0.2f, 0.2f );
	Ogre::ColourValue colour_magenta( 1.0f, 0.0f, 1.0f );
	Ogre::ColourValue colour_yellow ( 1.0f, 1.0f, 0.0f );
	Ogre::ColourValue colour_cyan   ( 0.0f, 1.0f, 1.0f );
	const float s = 0.5f;
	const float z = 0.1f;
	for( unsigned int i = 0; i < nvg->num_spawns; i++ ) {
		nav::veh::Node &o = nvg->nodes[i+1];
		graph->position( o.x-s, o.y-s, o.z+z );  graph->colour( colour_magenta );
		graph->position( o.x+s, o.y+s, o.z+z );  graph->colour( colour_magenta );
		graph->position( o.x-s, o.y+s, o.z+z );  graph->colour( colour_magenta );
		graph->position( o.x+s, o.y-s, o.z+z );  graph->colour( colour_magenta );
	}

	const Ogre::ColourValue *color1 = &colour_gray;
	const Ogre::ColourValue *color2 = &colour_gray;
	for( unsigned int i = 1; i < nvg->num_nodes; i++ ) {
		nav::veh::Node &o = nvg->nodes[i];
		switch( o.from[0].sign ) {
			case nav::veh::sign::SPAWN:  color1 = &colour_magenta;  break;
			default:                     color1 = &colour_gray;     break;
		}
		for( int n = 0; n < 2; n++ ) {
			if( o.next[n] ) {
				nav::veh::Node &t = nvg->nodes[ o.next[n] ];
				assert( i == t.prev[0] || i == t.prev[1] );
				switch( t.from[ i == t.prev[1] ? 1 : 0 ].sign ) {
					case nav::veh::sign::NONE:      color2 = &Ogre::ColourValue::White;   break;
					case nav::veh::sign::SPAWN:     color2 = &colour_magenta;             break;
					case nav::veh::sign::YIELD:     color2 = &Ogre::ColourValue::Blue;    break;
					case nav::veh::sign::STOP:      color2 = &Ogre::ColourValue::Red;     break;
					case nav::veh::sign::SEMAPHORE: color2 = &colour_yellow;              break;
					case nav::veh::sign::SPEED:     color2 = &colour_cyan;                break;
					default: assert( !"SimulationBuildVehicleGraph: Invalid node.from[?].sign" );
				}
				graph->position( o.x, o.y, o.z+z ); graph->colour( *color1 );
				graph->position( t.x, t.y, t.z+z ); graph->colour( *color2 );
			}
		}
	}
	
	graph->end();

	Ogre::SceneNode *node = scene_mgr->getRootSceneNode()->createChildSceneNode( "veh_graph_" );
	node->attachObject( graph );
}


/// Genera la represenciación visual del grafo de navegación de peatones.
/// Cada nodo es pintado de un color diferente, en función de la señal de tráfico.
static void BuildPedestrianGraph( const vis::Config &config, Ogre::SceneManager *scene_mgr )
{
	const nav::ped::Graph *npg = sim::GetNavigationPedestrianGraph();
	
	Ogre::ManualObject *graph = scene_mgr->createManualObject( "ped_graph_" );
	graph->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
		
	Ogre::ColourValue colour_gray   ( 0.2f, 0.2f, 0.2f );
	Ogre::ColourValue colour_magenta( 1.0f, 0.0f, 1.0f );
	Ogre::ColourValue colour_yellow ( 1.0f, 1.0f, 0.0f );
	const float s = 0.5f;
	const float z = 0.1f;
	for( unsigned int i = 0; i < npg->num_spawns; i++ ) {
		nav::ped::Node &o = npg->nodes[i+1];
		graph->position( o.x-s, o.y-s, o.z+z );  graph->colour( colour_magenta );
		graph->position( o.x+s, o.y+s, o.z+z );  graph->colour( colour_magenta );
		graph->position( o.x-s, o.y+s, o.z+z );  graph->colour( colour_magenta );
		graph->position( o.x+s, o.y-s, o.z+z );  graph->colour( colour_magenta );
	}

	const Ogre::ColourValue *color1 = &colour_gray;
	const Ogre::ColourValue *color2 = &colour_gray;
	for( unsigned int i = 1; i < npg->num_nodes; i++ ) {
		nav::ped::Node &o = npg->nodes[i];
		switch( o.sign ) {
			case nav::ped::sign::NONE:       color1 = &colour_gray;     break;
			case nav::ped::sign::SPAWN:      color1 = &colour_magenta;  break;
			case nav::ped::sign::SEMAPHORE:  color1 = &colour_yellow;   break;
			default: assert( !"SimulationBuildPedestrianGraph: Invalid Node.sign" );
		}
		for( int j = 0; j < 4; j++ ) {
			if( o.na[j].next ) {
				nav::ped::Node &t = npg->nodes[ o.na[j].next ];
				assert( i == t.na[0].next || i == t.na[1].next || i == t.na[2].next || i == t.na[3].next );
				switch( t.sign ) {
					case nav::ped::sign::NONE:       color2 = &colour_gray;    break;
					case nav::ped::sign::SPAWN:      color2 = &colour_magenta; break;
					case nav::ped::sign::SEMAPHORE:  color2 = &colour_yellow;  break;
					default: assert( !"SimulationBuildPedestrianGraph: Invalid Node.sign" );
				}
				graph->position( o.x, o.y, o.z+z ); graph->colour( *color1 );
				graph->position( t.x, t.y, t.z+z ); graph->colour( *color2 );
			}
		}
	}
	
	graph->end();

	Ogre::SceneNode *node = scene_mgr->getRootSceneNode()->createChildSceneNode( "ped_graph_" );
	node->attachObject( graph );
}


/// Genera la representación visual de cada vehículo.
/// El modelo del vehículo es genérico, cambiando solamente la escala en proporción a la medida del vehículo.
static void BuildVehicleNodes( const vis::Config &config, Ogre::SceneManager *scene_mgr )
{
	Ogre::ManualObject *circle_red = scene_mgr->createManualObject( "circle_red" );
	circle_red->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN );
	circle_red->colour( 1.0f, 0.0f, 0.0f );
	circle_red->position( 0.0f, 0.0f, 0.0f );
	for( int i = 0; i < 16+1; i++ ) {
		float rads = 2 * Ogre::Math::PI * i / 16;
		circle_red->position( 1.0f*Ogre::Math::Cos(rads), 1.0f*Ogre::Math::Sin(rads), 0.0f );
	}
	circle_red->end();
	circle_red->convertToMesh( "circle_red.mesh" );

	Ogre::ManualObject *circle_yellow = scene_mgr->createManualObject( "circle_yellow" );
	circle_yellow->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN );
	circle_yellow->colour( 0.5f, 0.5f, 0.0f );
	circle_yellow->position( 0.0f, 0.0f, 0.0f );
	for( int i = 0; i < 16+1; i++ ) {
		float rads = 2 * Ogre::Math::PI * i / 16;
		circle_yellow->position( 1.0f*Ogre::Math::Cos(rads), 1.0f*Ogre::Math::Sin(rads), 0.0f );
	}
	circle_yellow->end();
	circle_yellow->convertToMesh( "circle_yellow.mesh" );

	const int num_vehicles = sim::GetNumVehicles();
	const sim::Vehicle *sim_vehicles = sim::GetVehicles();

	vehicles.clear();
	vehicles.resize( num_vehicles );

	for( int i = 0; i < num_vehicles; i++ )
	{
		const sim::Vehicle &sim_veh = sim_vehicles[i];
		vis::Vehicle       &vis_veh = vehicles[i];

		Ogre::String name = Ogre::String("VehicleEnt") + Ogre::StringConverter::toString(i);
		Ogre::Entity *ent = scene_mgr->createEntity( name, config.veh_mesh );
		vis_veh.node = scene_mgr->getRootSceneNode()->createChildSceneNode( name );
		vis_veh.node->setPosition( sim_veh.px, sim_veh.py, sim_veh.pz );
		vis_veh.node->setScale( sim_veh.length/2, sim_veh.width/2, 1.6f/2 );
		vis_veh.node->attachObject( ent );
		
		if( i < 4 ) {
			name = Ogre::String("VehicleEllipse") + Ogre::StringConverter::toString(i);
			ent = scene_mgr->createEntity( name, "circle_yellow.mesh" );
			vis_veh.ellipse = scene_mgr->getRootSceneNode()->createChildSceneNode( name );
			vis_veh.ellipse->setPosition( sim_veh.px, sim_veh.py, sim_veh.pz-0.25f );
			vis_veh.ellipse->setScale( 1.0f, 1.0f, 1.0f );
			vis_veh.ellipse->attachObject( ent );
		}

		if( i < 4*100000 ) {
			name = Ogre::String("VehicleCollision") + Ogre::StringConverter::toString(i);
			ent = scene_mgr->createEntity( name, "circle_red.mesh" );
			vis_veh.collision = scene_mgr->getRootSceneNode()->createChildSceneNode( name );
			vis_veh.collision->setPosition( sim_veh.px, sim_veh.py, sim_veh.pz-0.20f );
			vis_veh.collision->setScale( 1.0f, 1.0f, 1.0f );
			vis_veh.collision->attachObject( ent );
		}
	}

}


/// Genera la representación visual de cada peatón.
/// El modelo del peatón es genérico, cambiando solamente la escala en proporción al radio del peatón.
static void BuildPedestrianNodes( const vis::Config &config, Ogre::SceneManager *scene_mgr )
{
	const int num_pedestrians = sim::GetNumPedestrians();
	const sim::Pedestrian *sim_pedestrians = sim::GetPedestrians();

	pedestrians.clear();
	pedestrians.resize( num_pedestrians );
	
	for( int i = 0; i < num_pedestrians; i++ )
	{
		const sim::Pedestrian &sim_ped = sim_pedestrians[i];
		vis::Pedestrian       &vis_ped = pedestrians[i];
		
		Ogre::String name = Ogre::String("PedestrianEnt") + Ogre::StringConverter::toString(i);
		Ogre::Entity *ent = scene_mgr->createEntity( name, config.ped_mesh );
		vis_ped.node = scene_mgr->getRootSceneNode()->createChildSceneNode( name );
		vis_ped.node->setPosition( sim_ped.px, sim_ped.py, sim_ped.pz );
		vis_ped.node->setScale( sim_ped.radius, sim_ped.radius, sim_ped.height );
		vis_ped.node->attachObject( ent );
	}
}


/// Actualiza los datos de visualización del vehículo.
/// \param [in,out]  vis_veh  Información visual del vehículo a actualizar.
/// \param [in]      sim_veh  Información del estado de simulación del vehículo.
static void VehicleRender( vis::Vehicle &vis_veh, const sim::Vehicle &sim_veh )
{
	const float s = 1.0f / 0x7FFF;
	vis_veh.node->setPosition( sim_veh.px, sim_veh.py, sim_veh.pz );
	//vis_veh.node->setDirection( sim_veh.dx, sim_veh.dy, 0.0f, Ogre::SceneNode::TS_WORLD, Ogre::Vector3::UNIT_Y );
	vis_veh.node->setOrientation( Ogre::Quaternion( sim_veh.orientation[3]*s, sim_veh.orientation[0]*s, sim_veh.orientation[1]*s, sim_veh.orientation[2]*s ) );

	const float dist_max     = sim_veh._dist_max;
	const float eccentricity = sim_veh._eccentricity;
	if( vis_veh.ellipse && dist_max > 0.1f ) {
		const float xmax = +dist_max * (1-eccentricity);
		const float xmin = -dist_max * (1-eccentricity);
		const float ymax = +dist_max * (1-eccentricity)/(1-eccentricity);
		const float ymin = -dist_max * (1-eccentricity)/(1+eccentricity);
		const float tx = sim_veh.dx * ( 0.5f*(ymax-ymin) + ymin );
		const float ty = sim_veh.dy * ( 0.5f*(ymax-ymin) + ymin );
		vis_veh.ellipse->setPosition( sim_veh.px+tx, sim_veh.py+ty, sim_veh.pz+0.1f );
		vis_veh.ellipse->setDirection( sim_veh.dx, sim_veh.dy, 0.0f, Ogre::SceneNode::TS_WORLD, Ogre::Vector3::UNIT_Y );
		vis_veh.ellipse->setScale( 0.5f*(xmax-xmin), 0.5f*(ymax-ymin), 1.0f );
	}	
	
	if( vis_veh.collision ) {
		vis_veh.collision->setDirection( Ogre::Vector3(1,0,0), Ogre::Node::TS_WORLD, Ogre::Vector3::UNIT_X );
		vis_veh.collision->setScale( 0.6f*sim_veh.width, 0.6f*sim_veh.width, 1.0f );
		vis_veh.collision->setPosition( sim_veh.px, sim_veh.py, sim_veh.pz+0.11f );
		if( sim_veh._other ) {
			Ogre::Vector3 a( sim_veh.px, sim_veh.py, sim_veh.pz+1.7f );
			Ogre::Vector3 b( sim_veh._other->px, sim_veh._other->py, sim_veh._other->pz+1.6f );
			Ogre::Vector3 r = b - 0.5f * ( a + b );
			const float s = r.normalise();
			vis_veh.collision->setScale( s, 0.2f, 1.0f );
			vis_veh.collision->setPosition( a + s * r );
			vis_veh.collision->setDirection( r, Ogre::Node::TS_WORLD, Ogre::Vector3::UNIT_X );
		} else if( sim_veh._colli_node ) {
			Ogre::Vector3 a( sim_veh.px, sim_veh.py, sim_veh.pz+0.12f );
			Ogre::Vector3 b( sim_veh._colli_node->x, sim_veh._colli_node->y, sim_veh.pz+0.12f );
			Ogre::Vector3 r = b - 0.5f * ( a + b );
			const float s = r.normalise();
			vis_veh.collision->setScale( s, 0.2f, 1.0f );
			vis_veh.collision->setPosition( a + s * r );
			vis_veh.collision->setDirection( r, Ogre::Node::TS_WORLD, Ogre::Vector3::UNIT_X );
		}
	}
}


/// Actualiza los datos de visualización del peatón.
/// \param [in,out]  vis_ped  Información visual del peatón a actualizar.
/// \param [in]      sim_ped  Información del estado de simulación del peatón.
static void PedestrianRender( vis::Pedestrian &vis_ped, const sim::Pedestrian &sim_ped )
{
	vis_ped.node->setPosition( sim_ped.px, sim_ped.py, sim_ped.pz );
	vis_ped.node->setDirection( sim_ped.dx, sim_ped.dy, 0.0f, Ogre::SceneNode::TS_WORLD, Ogre::Vector3::UNIT_X );
	vis_ped.node->setScale( sim_ped.radius, sim_ped.radius, sim_ped.height ); //#### remove this
}


/// Reserva e inicializa los recursos visuales de las entidades de la simulación.
static void SceneInitialize( const vis::Config &config, Ogre::SceneManager *scene_mgr )
{
	BuildVehicleGraph( config, scene_mgr );
	BuildVehicleNodes( config, scene_mgr );
	
	BuildPedestrianGraph( config, scene_mgr );
	BuildPedestrianNodes( config, scene_mgr );

	//####BUS
	const sim::Bus &sim_bus = *sim::GetBus();
	vis::Bus       &vis_bus = bus;
	Ogre::String name = Ogre::String( "BusEnt0" );
	Ogre::Entity *ent = scene_mgr->createEntity( name, "bus.mesh" );
	/*Ogre::Entity *ent = scene_mgr->createEntity( name, "Bus_Visual.mesh" );*/
	vis_bus.node = scene_mgr->getRootSceneNode()->createChildSceneNode( name );
	vis_bus.node->setPosition( sim_bus.px, sim_bus.py, sim_bus.pz );
	vis_bus.node->setScale( sim_bus.length, sim_bus.width, sim_bus.height );
	vis_bus.node->attachObject( ent );
}


/// Libera los recursos visuales de las entidades de la simulación.
static void SceneFinalize( void )
{
	vehicles.clear();
	pedestrians.clear();
}


/// Actualiza todas las representaciónes de las entidades de la simulación.
/// \param [in]  dt   Paso de tiempo de simulación.
static void SceneUpdate( Ogre::SceneManager *scene_mgr, const float dt )
{
	const int num_vehicles = sim::GetNumVehicles();
	const sim::Vehicle *sim_vehicles = sim::GetVehicles();
	for( int i = 0; i < num_vehicles; i++ )
		VehicleRender( vehicles[i], sim_vehicles[i] );

	const int num_pedestrians = sim::GetNumPedestrians();
	const sim::Pedestrian *sim_pedestrians = sim::GetPedestrians();
	for( int i = 0; i < num_pedestrians; i++ )
		PedestrianRender( pedestrians[i], sim_pedestrians[i] );

	//####BUS
	const float s = 1.0f / 0x7FFF;
	const sim::Bus &sim_bus = *sim::GetBus();
	bus.node->setPosition( sim_bus.px, sim_bus.py, sim_bus.pz+sim_bus.height/2 );
	bus.node->setOrientation( Ogre::Quaternion( sim_bus.orientation[3]*s, sim_bus.orientation[0]*s, sim_bus.orientation[1]*s, sim_bus.orientation[2]*s ) );
	bus.node->setScale( sim_bus.length/2, sim_bus.width/2, sim_bus.height/2 );
}



// VIDEO STREAMING ////////////////////////////////////////


static const char *FFMPEG = "/home/idf/libraries/ffmpeg/ffmpeg";
//static const char *FFMPEG = "ffmpeg";
static const int   VIDEO_STREAM_FRAMERATE = 32;
static const char *VIDEO_STREAM_OUTPUT    = "video.avi";
static const char *VIDEO_STREAM_ENCODER   = "libx264"; //"mpeg4";
static const int   VIDEO_STREAM_KBITRATE  = 2000;

static void VideoStreaming( Ogre::RenderWindow *win=NULL )
{
	static FILE *file;
	static char *buff;
	static char *data;
	
	if( !win )
	{
		if( file ) fclose( file );
		if( buff ) ::free( buff );
		file = NULL;
		data = NULL;
		return;
	}
	
	const int w = win->getWidth();
	const int h = win->getHeight();

	if( !file )
	{
		char cmd[256];
		sprintf( cmd, "%s  -f rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i pipe:0  -an -c:v %s -r:v %d -b:v %dk  -y %s",
			FFMPEG, w, h, VIDEO_STREAM_FRAMERATE, VIDEO_STREAM_ENCODER, VIDEO_STREAM_FRAMERATE, VIDEO_STREAM_KBITRATE, VIDEO_STREAM_OUTPUT );
		file = popen( cmd, "w" );
		buff = (char*) ::malloc( w*h*3 + 32 );
		data = (char*) ( ( (intptr_t)buff + 31 ) & ~31 );	// mem align to 32B
		assert( file && buff && data );
	}
	
	Ogre::PixelBox box( w, h, 1, Ogre::PF_BYTE_RGB, data );

	win->copyContentsToMemory( box, Ogre::RenderTarget::FB_FRONT );

	int r = fwrite( data, 1, w*h*3, file );
	assert( r == w*h*3 );
}



///////////////////////////////////////////



/// Clase para iniciar la visualización del simulador en Ogre.
class Application : public BaseApplication
{
	public:

		Application( void );
		virtual ~Application( void );

	protected:

		virtual bool frameRenderingQueued( const Ogre::FrameEvent &evt );

		virtual bool keyPressed( const OIS::KeyEvent &arg );
		virtual bool keyReleased( const OIS::KeyEvent &arg );
		
		virtual void createScene( void );
};


Application::Application()
{
}


Application::~Application()
{
}


bool Application::frameRenderingQueued( const Ogre::FrameEvent &evt )
{
    if( this->closed() ) return false;

	return this->BaseApplication::frameRenderingQueued( evt );
}


bool Application::keyPressed( const OIS::KeyEvent &arg )
{
	switch( arg.key )
	{
		//case OIS::KC_I:  	break;
		//case OIS::KC_K:  	break;
		//case OIS::KC_J:  	break;
		//case OIS::KC_L:  	break;
		default: break;		
	}
	
	return this->BaseApplication::keyPressed( arg );
}


bool Application::keyReleased( const OIS::KeyEvent &arg )
{
 	switch( arg.key )
	{
		//case OIS::KC_SPACE:
		//	for( int i = 0; i < VEHICLES_MAX; i++ ) {
		//		vehicles[i].px += 0.5f * (rand()%2000-1000)/1000.0f;
		//		vehicles[i].py += 0.5f * (rand()%2000-1000)/1000.0f;
		//	}
		//break;
		
		default: break;
	}
	
	return this->BaseApplication::keyReleased( arg );
}


void Application::createScene( void )
{
	Ogre::Light *light = this->mSceneMgr->createLight( "MainLight" );
	light->setType( Ogre::Light::LT_DIRECTIONAL );
	light->setDiffuseColour( 1.0f, 1.0f, 1.0f );
	light->setDirection( 1.0f, 2.0f, -3.0f );
	
	this->mSceneMgr->setAmbientLight( Ogre::ColourValue( 0.25f, 0.25f, 0.25f ) );
}



/////////////////////////////////////////////////////////



static Application app;


void vis::Initialize( const Config &config )
{
	assert( sim::GetNavigationVehicleGraph() );
	assert( sim::GetNavigationPedestrianGraph() );
	assert( sim::GetVehicles() );
	assert( sim::GetPedestrians() );
		
    if( !app.initialize( config.cfg_resources, config.cfg_plugins, config.vsync ) ) return;
	
	SceneInitialize( config, app.getSceneManager() );
}


void vis::Finalize( void )
{
	//VideoStreaming();
	
	app.finalize();
	
	SceneFinalize();
}


bool vis::Update( const float dt )
{
	if( app.closed() ) return false;
	
	if( 0 ) //####VIDEO
	{
		VideoStreaming( app.getRenderWindow() );
		SceneUpdate( app.getSceneManager(), dt );
		//SceneUpdate( app.getSceneManager(), 1.0f/VIDEO_STREAM_FRAMERATE );
	}
	else
	{
		SceneUpdate( app.getSceneManager(), dt );
	}
	
	app.render( dt );
	
	return true;
}
