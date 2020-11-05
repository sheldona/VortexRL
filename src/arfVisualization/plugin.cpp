#include "ArfVisualizationExtension.h"

#include <VxSim/VxExtensionFactory.h>

#include <VxPluginSystem/VxPluginInfo.h>
#include <VxPluginSystem/VxPluginSystem.h>
#include <Vx/VxEnumTable.h>
#include <Vx/VxEnum.h>

#ifdef _MSC_VER
#define SYMBOL __declspec(dllexport)
#else
#define SYMBOL
#endif

using namespace arfVisualization;

namespace
{

}
extern "C" SYMBOL bool GetPluginInfo(VxPluginSystem::VxPluginInfo & pluginInfo)
{
    pluginInfo.setDisplayName("Arf Collision Geometry Visualization Extension");
    pluginInfo.setVendor("ETS");
    pluginInfo.setDescription("Draws collision geometries using solid colors, and shadows enabled.");
    pluginInfo.setGroup("Arf");

    return true;
}

extern "C" SYMBOL bool InitializePlugin(VxPluginSystem::VxPluginManager & pluginManager, int /*argc*/, char** /*argv*/, std::string* /*error*/)
{
    return VxSim::VxExtensionFactory::registerType<ArfVisualizationExtension>(kExtensionFactoryKey, pluginManager.getCurrentVXP()); 
    
}

extern "C" SYMBOL bool UninitializePlugin(VxPluginSystem::VxPluginManager & pluginManager, std::string* /*error*/)
{
    return VxSim::VxExtensionFactory::unregisterType(kExtensionFactoryKey);
}