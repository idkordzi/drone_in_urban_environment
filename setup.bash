if [ -z "$PROJECT_PATH" ];
then 
    PROJECT_PATH="$HOME/workspace/drone_in_urban_environment"

    export GZ_SIM_SYSTEM_PLUGIN_PATH=$PROJECT_PATH/build/drone_simulation/modules/ardupilot_gazebo:$PROJECT_PATH/build/drone_simulation:$GZ_SIM_SYSTEM_PLUGIN_PATH
    export GZ_SIM_RESOURCE_PATH=$PROJECT_PATH/drone_simulation/modules/ardupilot_gazebo/models:$PROJECT_PATH/drone_simulation/modules/ardupilot_gazebo/worlds:$PROJECT_PATH/drone_simulation/models:$PROJECT_PATH/drone_simulation/worlds:$GZ_SIM_RESOURCE_PATH
else 
    echo "Gazebo variables already set!"
fi

source $PROJECT_PATH/install/setup.bash
