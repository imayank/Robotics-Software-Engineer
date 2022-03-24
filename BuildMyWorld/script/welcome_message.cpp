#include <gazebo/gazebo.hh>

namespace gazebo
{
	class WelcomeMessagePlugin : public WorldPlugin
	{
		public: WelcomeMessagePlugin() : WorldPlugin()
		{

			printf("Welcome to robot's world\n");
		}

		public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
		{

		} 
	};

	GZ_REGISTER_WORLD_PLUGIN(WelcomeMessagePlugin)
}